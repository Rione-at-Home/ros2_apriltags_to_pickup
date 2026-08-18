# ROS 2 チャレンジノード – ArUcoマーカー検出＆ロボット仕分け

このプロジェクトは、カメラを使用して **ArUcoマーカー**を検出し、ロボットが対象物まで移動して把持し、マーカーIDに応じて左または右に仕分けるROS 2システムです。

システムは主に以下の2つのROS 2ノードで構成されています。

1. **`TagDetectorNode`** – カメラ画像からArUcoマーカーを検出するノード
2. **`ChallengeNode`** – ステートマシンを使用してロボットを制御するノード

---

# システム概要

ロボットは以下の手順で動作します。

```text
カメラ
  │
  ▼
TagDetectorNode
  │
  ├── /tag_pose ──► マーカーの位置
  │
  └── /tag_id ───► マーカーID
                       │
                       ▼
                ChallengeNode
                       │
          ┌────────────┴────────────┐
          │                         │
          ▼                         ▼
       ロボット本体              ロボットアーム
          │                         │
       接近動作                  把持・仕分け
          │                         │
          └────────────┬────────────┘
                       ▼
                    完了
```

ロボットは最初に回転しながらArUcoマーカーを探します。

マーカーを検出すると、比例制御（P制御）を使用してマーカーの方向に向きを合わせ、対象物まで移動します。

所定の距離まで近づくとアームで対象物を持ち上げ、検出したマーカーIDに応じて左または右に対象物を置きます。

---

# ノード

## 1. `TagDetectorNode`

`TagDetectorNode` は、カメラ画像からArUcoマーカーを検出するためのノードです。

主な役割：

* カメラ画像を受信
* ROS 2の画像メッセージをOpenCV画像に変換
* ArUcoマーカーを検出
* マーカーのカメラからの位置を推定
* マーカーの位置をパブリッシュ
* マーカーIDをパブリッシュ

### 入力

| トピック         | メッセージ型                  | 説明    |
| ------------ | ----------------------- | ----- |
| `/image_raw` | `sensor_msgs/msg/Image` | カメラ画像 |

### 出力

| トピック        | メッセージ型                          | 説明          |
| ----------- | ------------------------------- | ----------- |
| `/tag_pose` | `geometry_msgs/msg/PoseStamped` | 検出したマーカーの位置 |
| `/tag_id`   | `std_msgs/msg/Int32`            | 検出したマーカーのID |

---

# 2. `ChallengeNode`

`ChallengeNode` は、ロボット全体の動作を制御するメインノードです。

`TagDetectorNode` からマーカーの位置とIDを受け取り、以下の動作を制御します。

* ロボット本体の移動
* ロボットアーム
* マーカー探索
* マーカーへの接近
* 対象物の把持
* 対象物の仕分け
* チャレンジ完了

---

# ステートマシン

チャレンジは以下の5つの状態で構成されています。

```text
             ┌─────────────┐
             │  SEARCHING  │
             │   探索中     │
             └──────┬──────┘
                    │
              マーカー検出
                    │
                    ▼
             ┌─────────────┐
             │ APPROACHING │
             │   接近中     │
             └──────┬──────┘
                    │
             所定距離に到達
                    │
                    ▼
              ┌─────────┐
              │ PICKING │
              │   把持    │
              └────┬────┘
                   │
                 把持完了
                   │
                   ▼
              ┌─────────┐
              │ SORTING │
              │   仕分け  │
              └────┬────┘
                   │
               仕分け完了
                   │
                   ▼
             ┌──────────┐
             │ FINISHED │
             │   完了    │
             └──────────┘
```

---

## `SEARCHING` – マーカー探索

ロボットはゆっくり回転しながらArUcoマーカーを探します。

```python
self.robot.base.drive(linear=0.0, angular=0.3)
```

マーカーが検出されると、ロボットは停止して `APPROACHING` 状態へ移行します。

---

## `APPROACHING` – マーカーへの接近

ロボットはP制御を使用して、マーカーに対して以下を行います。

1. マーカーの左右方向に位置合わせ
2. マーカーに向かって前進
3. 所定の距離に到達したら停止

角速度は以下の式で計算されます。

```python
angular_speed = -self.KP_ANGULAR * self.target_x_offset
```

距離の誤差は以下のように計算されます。

```python
dist_error = self.target_z_dist - self.PICKUP_DISTANCE
```

その後、前進速度を計算します。

```python
linear_speed = self.KP_LINEAR * dist_error
```

速度には安全のため上限が設定されています。

```python
linear_speed = max(0.0, min(0.2, linear_speed))
angular_speed = max(-0.4, min(0.4, angular_speed))
```

デフォルトの停止距離は：

```python
PICKUP_DISTANCE = 0.35
```

です。

距離誤差が `0.02 m` 以下になると、ロボットは停止して `PICKING` 状態へ移行します。

---

## `PICKING` – 対象物の把持

ロボットアームを使用して対象物を把持します。

現在のコードでは以下の処理を行います。

```python
self.robot.arm.pick_can()
self.robot.arm.lift()
```

つまり、

1. 対象物を把持
2. アームを持ち上げる

という順番で動作します。

処理が完了すると `SORTING` 状態へ移行します。

---

## `SORTING` – 対象物の仕分け

検出したArUcoマーカーのIDによって、対象物を置く場所を決定します。

| マーカーID | 仕分け先     |
| -----: | -------- |
|    `0` | 左        |
|    `1` | 右        |
|    その他 | 左（デフォルト） |

### ID `0` の場合

```python
self.robot.arm.place_left()
```

左側に対象物を置きます。

### ID `1` の場合

```python
self.robot.arm.place_right()
```

右側に対象物を置きます。

仕分けが完了すると、アームをホームポジションに戻します。

```python
self.robot.arm.home()
```

その後、`FINISHED` 状態へ移行します。

---

## `FINISHED` – チャレンジ完了

ミッションが完了すると、ロボット本体を停止します。

```python
self.robot.base.stop()
```

その後、以下のログを表示します。

```text
Mission complete!
```

---

# パラメータと制御値

`ChallengeNode` では、以下の制御パラメータを使用しています。

| パラメータ             |      デフォルト値 | 説明             |
| ----------------- | ----------: | -------------- |
| `PICKUP_DISTANCE` |    `0.35 m` | 把持を行うために停止する距離 |
| `KP_ANGULAR`      |       `0.8` | 角速度のPゲイン       |
| `KP_LINEAR`       |       `0.4` | 前進速度のPゲイン      |
| 最大直進速度            |   `0.2 m/s` | 前進速度の上限        |
| 最大角速度             | `0.4 rad/s` | 回転速度の上限        |
| 制御周波数             |     `10 Hz` | メイン制御ループの実行周波数 |

---

# ArUcoマーカー検出

画像処理にはOpenCVのArUco検出機能を使用しています。

使用している辞書は：

```python
cv2.aruco.DICT_4X4_50
```

です。

---

## マーカーサイズ

マーカーのサイズは以下のように設定されています。

```python
MARKER_SIZE = 0.05
```

これは、

```text
50 mm = 0.05 m
```

を意味します。

実際に使用するマーカーのサイズと、この値が一致している必要があります。

---

# マーカーの位置推定

ArUcoマーカーを検出すると、カメラを基準としたマーカーの位置を推定します。

位置は以下の3つの値で表されます。

```text
[x, y, z]
```

それぞれ：

```text
x = カメラから見た左右方向の位置
y = 上下方向の位置
z = カメラからマーカーまでの距離
```

を表します。

`ChallengeNode` では主に `x` と `z` を使用します。

---

# カメラ設定

現在のコードでは、標準的なWebカメラを想定した近似的なカメラ内部パラメータを使用しています。

```python
camera_matrix = [
    [600,   0, 320],
    [  0, 600, 240],
    [  0,   0,   1]
]
```

歪み係数は現在、ゼロとして設定されています。

```python
dist_coeffs = np.zeros((4, 1), dtype=np.float32)
```

これは簡易的な設定です。

実際のロボットでより正確な距離推定を行う場合は、カメラキャリブレーションを行い、実際のカメラ内部パラメータと歪み係数を使用することを推奨します。

---

# ROS 2 トピック

ノード間の通信は以下のようになっています。

```text
/image_raw
     │
     ▼
TagDetectorNode
     │
     ├──────────────► /tag_pose
     │
     └──────────────► /tag_id
                          │
                          ▼
                    ChallengeNode
                          │
                          ▼
                       Robot
```

---

## `/image_raw`

**メッセージ型：**

```text
sensor_msgs/msg/Image
```

カメラから取得した画像を受信します。

`TagDetectorNode` がこのトピックを購読します。

---

## `/tag_pose`

**メッセージ型：**

```text
geometry_msgs/msg/PoseStamped
```

検出したArUcoマーカーの位置を送信します。

主に以下の値が使用されます。

```text
x = 左右方向のずれ
z = マーカーまでの距離
```

---

## `/tag_id`

**メッセージ型：**

```text
std_msgs/msg/Int32
```

検出したArUcoマーカーのIDを送信します。

現在の仕分けルールは：

```text
ID 0 → 左
ID 1 → 右
```

です。

---

# 必要な依存パッケージ

このプロジェクトでは、以下のパッケージ・ライブラリが必要です。

* ROS 2
* Python 3
* `rclpy`
* OpenCV
* NumPy
* `cv_bridge`
* `geometry_msgs`
* `sensor_msgs`
* `std_msgs`

また、ロボット固有の `Robot` クラスが必要です。

```python
from .robot import Robot
```

`Robot` クラスには、少なくとも以下のようなインターフェースが必要です。

```python
robot.base.drive()
robot.base.stop()

robot.arm.pick_can()
robot.arm.lift()
robot.arm.place_left()
robot.arm.place_right()
robot.arm.home()
```

---

# 推奨パッケージ構成

ROS 2のPythonパッケージとして、例えば以下のような構成にできます。

```text
your_package/
├── package.xml
├── setup.py
├── resource/
│   └── your_package
└── your_package/
    ├── __init__.py
    ├── challenge_node.py
    ├── tag_detector_node.py
    └── robot.py
```

---

# ノードの起動

まずROS 2をsourceします。

```bash
source /opt/ros/<your_ros_distro>/setup.bash
```

次にワークスペースをビルドします。

```bash
cd ~/your_ros2_workspace
colcon build
```

ビルド後、ワークスペースをsourceします。

```bash
source install/setup.bash
```

カメラを起動し、以下のトピックがパブリッシュされていることを確認します。

```text
/image_raw
```

その後、ArUcoマーカー検出ノードを起動します。

```bash
ros2 run <your_package> tag_detector_node
```

別のターミナルを開き、もう一度ワークスペースをsourceします。

```bash
source ~/your_ros2_workspace/install/setup.bash
```

その後、チャレンジノードを起動します。

```bash
ros2 run <your_package> challenge_node
```

---

# ROS 2 トピックの確認

カメラ画像が正常に送信されているか確認するには：

```bash
ros2 topic echo /image_raw
```

ArUcoマーカーの位置を確認するには：

```bash
ros2 topic echo /tag_pose
```

マーカーIDを確認するには：

```bash
ros2 topic echo /tag_id
```

現在使用されているトピック一覧を確認するには：

```bash
ros2 topic list
```

---

# 動作確認の流れ

実際の動作は以下のようになります。

```text
1. ロボット起動
       ↓
2. ArUcoマーカーを探索
       ↓
3. マーカーを検出
       ↓
4. マーカーの中心に位置合わせ
       ↓
5. マーカーに向かって前進
       ↓
6. 把持可能な距離で停止
       ↓
7. 対象物を把持
       ↓
8. アームを持ち上げる
       ↓
9. マーカーIDを確認
       ↓
10. ID 0 → 左に配置
    ID 1 → 右に配置
       ↓
11. アームをホームポジションへ戻す
       ↓
12. チャレンジ完了
```

---

# 注意事項

## ArUcoマーカーが検出されない場合

まず、カメラ画像が正常に送信されているか確認してください。

```bash
ros2 topic list
```

また：

```bash
ros2 topic echo /image_raw
```

を使用して画像トピックを確認できます。

以下も確認してください。

* カメラが正常に動作しているか
* マーカーが `DICT_4X4_50` の辞書で作成されているか
* マーカーがカメラから見えているか
* マーカーが画像内で十分な大きさになっているか
* 照明が十分か

---

## ロボットがずっと回転している場合

ロボットは以下の条件で `SEARCHING` 状態を維持します。

```python
self.target_visible == False
```

そのため、`TagDetectorNode` が有効なマーカー情報を送信していない場合、ロボットは探索のために回転し続けます。

以下を確認してください。

```bash
ros2 topic echo /tag_pose
```

---

## ロボットの接近方向がおかしい場合

ロボットの接近動作は以下の値に依存します。

```python
target_x_offset
target_z_dist
```

角速度は以下の式で計算されています。

```python
angular_speed = -self.KP_ANGULAR * self.target_x_offset
```

もしロボットがマーカーと逆方向に旋回する場合、カメラの座標系とロボットの座標系におけるX軸の向きを確認してください。

必要に応じて符号を反転します。

---

## 距離が正確でない場合

マーカーまでの距離は主に以下の値に依存します。

```python
MARKER_SIZE
camera_matrix
dist_coeffs
```

特に `MARKER_SIZE` は実際のマーカーサイズと正確に一致させる必要があります。

より正確な距離推定を行うには、カメラキャリブレーションを実施することを推奨します。

---

# 座標系について

検出したマーカーの座標には以下のフレームが設定されています。

```python
pose_msg.header.frame_id = "camera_frame"
```

現在の実装では、カメラ座標系の `x` と `z` の値をそのままロボットの制御に使用しています。

そのため、カメラがロボットに対して斜めに取り付けられていたり、カメラ位置がロボットの中心から大きくずれている場合は、より正確な制御のために座標変換が必要になる可能性があります。

将来的にはTF2などを使用して、`camera_frame` からロボットのベース座標系へ変換することができます。

---

# 現在の実装上の制限

## 複数マーカーへの対応

現在のコードでは、複数のマーカーが検出された場合でも、**最初に検出されたマーカーのみ**を使用します。

```python
tvec = tvecs[0][0]
```

また、IDについても最初のマーカーを使用しています。

```python
id_msg.data = int(ids[0][0])
```

そのため、複数のマーカーが同時に見えている場合、最も近いマーカーや特定のIDを持つマーカーを選択する処理はありません。

---

## マーカーが見えなくなった場合

`ChallengeNode` は制御ループの最後で、

```python
self.target_visible = False
```

にリセットします。

そのため、画像処理ノードが継続的に `/tag_pose` を送信しない場合、マーカーが見えなくなったと判断されます。

接近中にマーカーを見失った場合、ロボットは `SEARCHING` 状態へ戻ります。

---

# 今後の改善案

今後、以下のような機能を追加すると、より安定したシステムになります。

* 実際のカメラキャリブレーションを使用する
* 検出したArUcoマーカーを画像上に表示する
* 複数のマーカーに対応する
* 最も近いマーカーを選択する
* マーカーを見失った場合のタイムアウト処理を追加する
* PゲインをROS 2パラメータとして変更可能にする
* `ERROR` 状態を追加する
* 緊急停止処理を追加する
* TF2を使用してカメラ座標からロボット座標へ変換する
* アームの動作が制御ループによって複数回実行されないようにする
* Launchファイルを作成し、カメラ・検出ノード・チャレンジノードをまとめて起動できるようにする

---

# まとめ

このプロジェクトでは、ROS 2とOpenCVを組み合わせて、カメラによるArUcoマーカー検出とロボット制御を行っています。

`TagDetectorNode` がカメラ画像からマーカーの位置とIDを検出し、その情報をROS 2トピックを通して `ChallengeNode` に送信します。

`ChallengeNode` はステートマシンに基づいてロボットを制御し、

```text
探索 → 接近 → 把持 → 仕分け → 完了
```

という一連の動作を実行します。

マーカーIDによって、

```text
ID 0 → 左
ID 1 → 右
```

に仕分けることができます。

---

# License

このプロジェクトに使用するライセンスをここに記載してください。

例：

```text
MIT License
```

または、授業・コンテスト・プロジェクトで指定されているライセンスを使用してください。
