# pcd_range_publisher

ROS2パッケージ：PCDマップを読み込んで、自己位置周辺の点群を一定間隔でpublishします。

ROS2 package that loads a PCD map and publishes surrounding pointcloud based on robot's position at regular intervals.

## 特徴 / Features

- **メモリ効率**: Octreeを使用した空間インデックスにより、大規模な点群マップでもメモリ効率的に処理
- **解像度調整**: VoxelGridフィルタにより、publishする点群の解像度を調整可能
- **パラメータ化**: 範囲、頻度、解像度などを柔軟に設定可能
- **C++実装**: 高速な処理のためにC++で実装

- **Memory Efficient**: Uses Octree spatial indexing for efficient processing of large pointcloud maps
- **Resolution Control**: VoxelGrid filter allows adjusting the resolution of published pointclouds
- **Parameterized**: Flexible configuration of range, frequency, and resolution
- **C++ Implementation**: Implemented in C++ for high performance

## 依存関係 / Dependencies

- ROS2 (Humble or later recommended)
- PCL (Point Cloud Library)
- pcl_ros
- tf2

## ビルド / Build

```bash
# ワークスペースのsrcディレクトリにクローン
cd ~/ros2_ws/src
git clone <this-repository>

# ビルド
cd ~/ros2_ws
colcon build --packages-select pcd_range_publisher

# セットアップ
source install/setup.bash
```

## 使い方 / Usage

### 基本的な使い方 / Basic Usage

1. `config/params.yaml` を編集して、PCDファイルのパスとパラメータを設定
2. launchファイルを実行

```bash
ros2 launch pcd_range_publisher pcd_range_publisher.launch.py
```

### コマンドラインからパラメータを指定 / With Command Line Parameters

```bash
ros2 launch pcd_range_publisher pcd_range_publisher.launch.py \
  pcd_file:=/path/to/your/map.pcd \
  publish_rate:=2.0 \
  range_radius:=100.0 \
  voxel_leaf_size:=0.1
```

### 直接ノードを起動 / Run Node Directly

```bash
ros2 run pcd_range_publisher pcd_range_publisher_node \
  --ros-args \
  -p pcd_file_path:=/path/to/your/map.pcd \
  -p publish_rate:=1.0 \
  -p range_radius:=50.0 \
  -p voxel_leaf_size:=0.2
```

## パラメータ / Parameters

| パラメータ名 / Parameter | デフォルト値 / Default | 説明 / Description |
|-------------------------|----------------------|-------------------|
| `pcd_file_path` | (required) | PCDファイルのパス / Path to PCD map file |
| `publish_rate` | 1.0 | publish頻度（Hz） / Publishing rate in Hz |
| `range_radius` | 50.0 | 自己位置周辺の抽出範囲（m） / Radius around robot position (meters) |
| `voxel_leaf_size` | 0.2 | ボクセルグリッドのサイズ（m）、0.0で無効化 / Voxel grid leaf size (meters), 0.0 to disable |
| `map_frame_id` | "map" | マップのフレームID / Map frame ID |
| `base_link_frame` | "base_link" | ロボットのフレームID / Robot frame ID |
| `octree_resolution` | 1.0 | Octreeの解像度（m） / Octree resolution (meters) |

## トピック / Topics

### Published Topics

- `~/surrounding_pointcloud` (`sensor_msgs/PointCloud2`)
  - 自己位置周辺の点群 / Surrounding pointcloud around robot position

### Subscribed Topics

- `/tf`, `/tf_static`
  - ロボットの自己位置を取得するために使用 / Used to get robot position

## メモリ使用量の最適化 / Memory Optimization

このパッケージは以下の工夫でメモリ使用量を抑えています：

This package optimizes memory usage through:

1. **Octree空間インデックス / Octree Spatial Indexing**
   - 全点群を毎回検索せず、Octreeで効率的に周辺点を抽出
   - Instead of searching all points, efficiently extract nearby points using Octree

2. **VoxelGridフィルタ / VoxelGrid Filtering**
   - 点群の解像度を下げることで、転送データ量を削減
   - Reduce data transfer by downsampling pointcloud resolution

3. **範囲フィルタリング / Range Filtering**
   - 必要な範囲のみを抽出してpublish
   - Only extract and publish points within necessary range

## パラメータ調整のヒント / Parameter Tuning Tips

### メモリ使用量を抑える / Reduce Memory Usage

- `octree_resolution` を大きくする（例: 2.0）
- `voxel_leaf_size` を大きくする（例: 0.5）
- `range_radius` を小さくする（例: 30.0）

- Increase `octree_resolution` (e.g., 2.0)
- Increase `voxel_leaf_size` (e.g., 0.5)
- Decrease `range_radius` (e.g., 30.0)

### 点群の品質を上げる / Improve Pointcloud Quality

- `voxel_leaf_size` を小さくする（例: 0.1）
- `range_radius` を大きくする（例: 100.0）
- `publish_rate` を上げる（例: 5.0）

- Decrease `voxel_leaf_size` (e.g., 0.1)
- Increase `range_radius` (e.g., 100.0)
- Increase `publish_rate` (e.g., 5.0)

### CPU使用量を抑える / Reduce CPU Usage

- `publish_rate` を下げる（例: 0.5）
- `voxel_leaf_size` を大きくする
- `octree_resolution` を大きくする

- Decrease `publish_rate` (e.g., 0.5)
- Increase `voxel_leaf_size`
- Increase `octree_resolution`

## トラブルシューティング / Troubleshooting

### ノードが起動しない / Node fails to start

- PCDファイルのパスが正しいか確認
- PCLライブラリがインストールされているか確認

- Check if PCD file path is correct
- Verify PCL library is installed

### 点群がpublishされない / No pointcloud published

- TF（map→base_link）が正しく配信されているか確認
- `ros2 run tf2_ros tf2_echo map base_link` で確認

- Check if TF (map→base_link) is correctly published
- Verify with `ros2 run tf2_ros tf2_echo map base_link`

### メモリ不足 / Out of memory

- パラメータを調整してメモリ使用量を削減
- 特に `octree_resolution` と `voxel_leaf_size` を大きくする

- Adjust parameters to reduce memory usage
- Particularly increase `octree_resolution` and `voxel_leaf_size`

## ライセンス / License

Apache-2.0

## 作者 / Author

Created with Claude Code
