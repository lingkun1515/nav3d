# OctoMap 地图格式与"占据"概念详解

## 1. OctoMap 核心数据结构

### 1.1 八叉树（OcTree）基础

OctoMap 是一个**概率占据八叉树**。三维空间被递归地八等分，形成一棵树：

```
根节点（整个空间）
├── 子节点 0（左下前1/8）
├── 子节点 1（右下前1/8）
├── ...
└── 子节点 7（右上后1/8）
     └── 继续递归到最大深度（resolution 决定）
```

**叶子节点（leaf）**：不再有子节点的节点，存储实际的占据信息。

**内部节点（inner node）**：有子节点的节点，本身不存储占据信息（其值由子节点推导）。

### 1.2 关键：占据信息不是布尔值，是 log-odds 概率

每个叶子节点存储的不是 `true/false`，而是一个 **log-odds 浮点值**：

| log-odds 值 | 概率 | 含义 |
|-------------|------|------|
| 1.5 | ~0.82 | 较确定被占据 |
| 0.847 | ~0.70 | 可能被占据（`updateNode(true)` 一次） |
| 0.0 | 0.50 | 完全未知 |
| -0.405 | ~0.40 | 可能空闲（`updateNode(false)` 一次） |
| -1.5 | ~0.18 | 较确定空闲 |

**判断"占据"的核心逻辑**（来自 OctoMap 源码）：

```cpp
// isNodeOccupied() ≈ 判断 log-odds >= 0
// 即：概率 >= 0.5 就视为 "占据"
bool isNodeOccupied(node) {
    return node->getLogOdds() >= 0;  // 计算得出的，不是直接存储的！
}
```

这意味着：**"占据"是一个从 log-odds 计算出来的判断，不是原始布尔标记。**

### 1.3 `updateNode` vs `setNodeValue` 的关键区别

```cpp
// updateNode(true): 累加式——给当前 log-odds 加上 +0.847（默认 prob_hit）
// updateNode(false): 累减式——减去 0.405（默认 prob_miss）
tree->updateNode(coord, true);   // log-odds += 0.847
tree->updateNode(coord, true);   // log-odds += 0.847（多次击中越来越确定）

// setNodeValue: 直接替换——不管之前是什么，直接设为指定值
tree->setNodeValue(coord, 1.5f);  // log-odds = 1.5，覆盖原有值
tree->setNodeValue(coord, -1.5f); // log-odds = -1.5，完全清除
```

**本项目已在 web 编辑中用 `setNodeValue` 替代 `updateNode`**，以确保编辑的确定性（`src/octo_planner/src/octo_planner_node.cpp:333-351`）。

---

## 2. 剪枝（Pruning）——为什么"占据格"不等于"原始数据点"

### 2.1 剪枝机制

OctoMap 有一个关键优化：**剪枝**。当某个内部节点的 8 个子节点**全部是叶子**且 log-odds 值**完全相同**时，这 8 个子节点被删除，父节点变成叶子节点，值设为相同的 log-odds。

```
剪枝前（8个独立叶子，depth=max）：          剪枝后（1个父叶子，depth=max-1）：
+---+---+                                  +-------+-------+
| A | A |   ← 每个A是独立叶子                 |               |
+---+---+                                  |       A       |  ← 一个叶子覆盖2x2x2
| A | A |    中心各不相同                    |               |
+---+---+                                  +-------+-------+
  实际占用 8 个存储节点                       只占用 1 个存储节点
```

**剪枝由 `updateInnerOccupancy()` 自动触发**——任何时候修改了叶子之后调用此函数，它会递归检查并合并。

### 2.2 剪枝对可视化的致命影响

剪枝后父节点的**中心坐标**与任何一个原始子节点的中心都不同：

```
假设 resolution = 0.2m，子节点 key = 0, 1（分别覆盖 [0, 0.2), [0.2, 0.4)）
  子节点 0 中心: (0 + 0.5) * 0.2 = 0.1
  子节点 1 中心: (1 + 0.5) * 0.2 = 0.3

剪枝后父节点 key = 0（depth-1），覆盖 [0, 0.4），size = 0.4
  父节点中心: (0 + 0.5) * 0.4 = 0.2  ← 既不是 0.1 也不是 0.3！
```

如果直接发布父节点中心作为"一个占据体素"，web 端会看到一个 0.4m 的方块在 0.2 位置，而不是两个 0.2m 方块在 0.1 和 0.3。

### 2.3 分解修复（当前实现）

`publish_occupied_markers()`（`octo_planner_node.cpp:571-611`）已实现分解：检测到剪枝节点时，将其展开为所有组成单元的中心坐标再发布。这样即使 octree 内部发生了剪枝，web 端看到的体素位置与原始数据一致。

**判断方法**：`it.getSize() > resolution` 表示这是一个剪枝节点（覆盖多个最小单元）。

---

## 3. 本项目完整数据流

### 3.1 地图加载 → 原始 OctoMap

```
PCD 点云文件
    │
    ▼
pcd2octomap_converter:
  1. loadPointCloud()
  2. buildVoxelCounts()        ← 统计每个体素内的点数
  3. filterByPointCount()       ← 剔除点数不足的体素
  4. filterByConnectedClusters()← 剔除孤立体素簇
  5. fillOcTree()               ← updateNode(true) 逐个写入
  6. groundInfill()             ← 地面补全（updateNode(true)）
  7. updateInnerOccupancy()     ← 剪枝合并
    │
    ▼
octree_ (std::shared_ptr<octomap::OcTree>)
    │
    ▼
configure_planner():
  - planner_->setOctomap(octree_)  ← planner 从此 octree 计算通行性等
  - publish_occupied_markers()     ← 从 octree 直接读取，发布占据体素
  - publish_traversable_markers()  ← 从 planner 的 traversable_cells_ 发布
  - publish_preblocked_markers()   ← 从 planner 的 preblocked_cells_ 发布
```

`.bt` 文件加载更简单：直接 `octree_ = make_shared<OcTree>(file_path)` 反序列化，然后同样调用 `configure_planner()`。

### 3.2 Web 编辑 → 更新原始 OctoMap

```
Web 前端（Three.js 本地渲染）
    │
    ├── applyLocalEdit()        ← 即时更新本地 occupiedPointsBuf + Three.js
    │
    └── flushEdits()
          │
          ├── buildPointCloud2(pendingAddPositions)
          │     └── Float32Array → Uint8Array → binary string → btoa → base64
          │
          ├── /add_occupied_voxels (PointCloud2 via rosbridge)
          │     └── on_add_voxels():
          │           setNodeValue(x, y, z, 1.5f)  ← 直接替换，确定性
          │           updateInnerOccupancy()        ← 可能触发剪枝
          │
          └── /remove_occupied_voxels (PointCloud2 via rosbridge)
                └── on_remove_voxels():
                      setNodeValue(x, y, z, -1.5f) ← 直接替换
                      updateInnerOccupancy()       ← 可能触发剪枝
```

### 3.3 退出编辑 → 重新发布

```
Web: flushEdits() → setTimeout(500ms) → requestMap()
                                              │
                                              ▼
                          ROS: /request_map 服务
                            ├── planner_->reanalyze()  ← 从 octree_ 重建所有衍生数据
                            └── republish_all()
                                  ├── publish_occupied_markers()   ← 含分解
                                  ├── publish_traversable_markers()
                                  ├── publish_preblocked_markers()
                                  └── publish_risk_cost_cloud()
                                              │
                                              ▼
                          Web: setOccupiedMarker() 等回调
                            └── 重建 occupiedPointsBuf → 重新渲染
```

---

## 4. 设计原则：原始 OctoMap 地图

当前架构已经满足你的设计原则：

- **"原始 octomap 地图" = `octree_`**（`std::shared_ptr<octomap::OcTree>`）
- **Web 编辑只修改 `octree_`**（通过 `setNodeValue` 直接写入）
- **所有衍生数据**（traversable_cells、preblocked_cells、costmap）**都是从 `octree_` 重新计算的**（`reanalyze()` / `setOctomap()` 均从 `octree_` 读取，不修改它）

`GlobalPlanner::reanalyze()` 和 `GlobalPlanner::setOctomap()` 只读取 octree，从不修改它。它们修改的是 `traversable_cells_`、`preblocked_cells_` 等 planner 内部成员变量。

---

## 5. 编辑前后不一致的潜在原因

既然架构是合理的，不一致可能来自以下环节：

### 5.1 PointCloud2 二进制编码（优先级：中）

`web/main.js:416-444` 的 `buildPointCloud2()`：

```javascript
const bytes = new Uint8Array(buf);
let binary = '';
for (let i = 0; i < bytes.length; i++) {
    binary += String.fromCharCode(bytes[i]);  // ← 字节→字符
}
// ...
data: btoa(binary)  // ← 字符→base64
```

理论上 `String.fromCharCode(byte)` 对 0-255 的值能正确生成 Latin-1 字符，`btoa` 也能正确编码。但**未经实地验证**。如果中间某个环节（rosbridge / DDS）对二进制数据做了额外的编解码转换，坐标可能会被破坏。

### 5.2 DDS 消息到达时序（优先级：中）

```
flushEdits() → 立即 publish PointCloud2 (异步)
setTimeout(500ms) → requestMap() → reanalyze + republish markers (同步)
```

虽然 500ms 的理论等待时间足够，但 rosbridge WebSocket → DDS 的桥接层可能引入额外延迟。如果 PointCloud2 消息在 `reanalyze()` 之后才到达 ROS 节点，编辑就不会被纳入。

### 5.3 Rosbridge Marker 消息丢失/乱序（优先级：低）

`publish_occupied_markers` 将大量体素分成多个 chunk（每个 5000 点），通过 rosbridge 发送。如果 marker 消息数量多（比如 10 万体素 = 20 个 chunk），在 WebSocket 层面可能出现丢包或乱序。`setOccupiedMarker` 依赖 `msg.id === 0` 来清空缓存，乱序会导致数据丢失。

### 5.4 World ↔ Grid 坐标映射差异（优先级：低）

`GlobalPlanner::rebuildDerivedLayers()` 用 `worldToGrid(it.getX(), it.getY(), it.getZ())` 将世界坐标转回格网索引。对于剪枝节点（即使已被分解修复处理了 occupied markers），planner 内部仍然只映射到一个格网索引而非全部组成单元。这影响 traversable/preblocked 层的精度，但不影响 occupied 层。

---

## 6. 总结

| 问题 | 答案 |
|------|------|
| 占据格是直接存储的吗？ | **不是**。存储的是 log-odds 概率值，"是否占据"是从 log-odds >= 0 计算得出的 |
| .bt 文件里的占据格和原始数据一一对应吗？ | **不一定**。PCD→OctoMap 经过了滤波、补全、剪枝，且 ground infill 会对同一体素多次 updateNode，log-odds 可能不同 |
| 剪枝会改变体素位置吗？ | **会**。已通过分解修复在 occupied markers 层面解决 |
| 编辑操作会正确写入 octree 吗？ | **会**。`setNodeValue` 直接替换 log-odds，确定性 |
| 为什么编辑前后还是不一致？ | 最可能的原因是 PointCloud2 编码或 DDS 时序问题，需要运行时日志验证 |

### 建议的调试步骤

1. 在 `on_add_voxels` 和 `on_remove_voxels` 中打印前几个接收到的坐标，对比 web 端发送的值
2. 在 `publish_occupied_markers` 中打印体素总数，对比 web 端 `occupiedPointsBuf.length`
3. 在 `/request_map` 服务处理前打印 `octree_->getNumLeafNodes()`，与 configure_planner 时的叶子数对比

这样可以确定不一致发生在"web→ROS 写入"阶段还是"ROS→web 读出"阶段。
