import pyxel
import math
import random # random.uniformを使うためにインポート

# Helper function for line-circle collision and response
# 直線と円の衝突判定と応答のためのヘルパー関数
# line_p1, line_p2: 線分の端点座標 (float)
# circle_c: 円の中心座標 (float)
# circle_r: 円の半径 (float)
# added_radius: 当たり判定半径に加算する値 (フリッパーの太さの半分など)
# ball_v: ボールの速度ベクトル (vx, vy) (float)
# bounce_factor: 反発係数
# flipper_v_at_collision: 衝突点におけるフリッパーの速度ベクトル (fx, fy) (float) - これを反射後の速度に加算
# 戻り値: 衝突した場合 (True, 新しい速度ベクトル, 新しいボール位置), 衝突しない場合 (False, 元の速度ベクトル, 元のボール位置)
def collide_line_circle(p1x, p1y, p2x, p2y, cx, cy, cr, added_radius, vx, vy, bounce_factor, flipper_v_at_collision_x=0.0, flipper_v_at_collision_y=0.0):
    # 線分のベクトル L = P2 - P1
    lx = p2x - p1x
    ly = p2y - p1y

    # 線分の長さの2乗
    len_sq = lx*lx + ly*ly
    is_point = (len_sq < 1e-6) # 非常に小さい値と比較して浮動小数点誤差を考慮

    # 当たり判定に使用する合計半径
    total_radius = cr + added_radius
    total_radius_sq = total_radius * total_radius

    # 円の中心から線分 P1P2 上への垂線の足 Q を求める (または P1, P2 の近い方)
    t = 0.0
    collision_point_x = p1x
    collision_point_y = p1y
    dist_sq = (cx - p1x)**2 + (cy - p1y)**2 # ボール中心から P1 までの距離の2乗

    if not is_point: # 線分の場合
        # パラメータ t = ((C - P1) . L) / |L|^2
        # P1とP2が同じ点の場合 len_sq は0になりゼロ除算になるため、is_point で分岐
        if len_sq > 1e-6: # ゼロ除算対策
             t = ((cx - p1x) * lx + (cy - p1y) * ly) / len_sq

        # Qの座標
        qx = p1x + t * lx
        qy = p1y + t * ly

        # Q が線分 P1P2 の範囲外にあるかどうか (0 <= t <= 1)
        # 微小量で範囲チェックを緩める
        if t < -1e-6 or t > 1.0 + 1e-6:
            # Q は線分上にはない -> 距離は C と P1 または C と P2 の近い方
            dist_sq_p1 = (cx - p1x)**2 + (cy - p1y)**2
            dist_sq_p2 = (cx - p2x)**2 + (cy - p2y)**2
            if dist_sq_p1 < dist_sq_p2:
                dist_sq = dist_sq_p1
                collision_point_x = p1x
                collision_point_y = p1y
            else:
                dist_sq = dist_sq_p2
                collision_point_x = p2x
                collision_point_y = p2y
        else: # Q が線分上にある (または非常に近い)
            dist_sq = (cx - qx)**2 + (cy - qy)**2
            collision_point_x = qx
            collision_point_y = qy

    # 距離が合計半径以下であれば衝突
    if dist_sq <= total_radius_sq:
        # print(f"  Potential collision detected at dist_sq={dist_sq:.4f}, total_r_sq={total_radius_sq:.4f}") # Debug
        dist_to_col = math.sqrt(dist_sq) # dist_sq >= 0 は保証されているはず

        # 衝突点における線分の法線ベクトル N (外側向き)
        if dist_to_col > 1e-6: # 衝突点が円の中心と重なっていない場合
            nx = (cx - collision_point_x) / dist_to_col
            ny = (cy - collision_point_y) / dist_to_col
        else: # 円の中心が衝突点と重なっている場合 (めり込み量が大きいなど)
             # 線分に垂直なベクトルを法線とする (または点の場合)
             if is_point: # 点の場合、中心からのベクトルを法線とする
                  nx = (cx - p1x) / (total_radius if total_radius > 1e-6 else 1.0) # 合計半径で割る
                  ny = (cy - p1y) / (total_radius if total_radius > 1e-6 else 1.0) # 合計半径で割る
                  mag_n = math.sqrt(nx*nx + ny*ny)
                  if mag_n > 1e-6: nx /= mag_n; ny /= mag_n # 正規化
                  else: nx=1.0; ny=0.0 # デフォルト

             else: # 線分の場合 (dist_to_col == 0 または非常に近い)
                  mag_l = math.sqrt(len_sq)
                  # 線分の方向ベクトル (lx, ly) に対して垂直なベクトルは (-ly, lx)
                  perp_lx = -ly / (mag_l if mag_l > 1e-6 else 1.0)
                  perp_ly = lx / (mag_l if mag_l > 1e-6 else 1.0)

                  # どちらの向きにするか？ ボールの中心 C が線分のどちら側にあるかで判断
                  # (C - P1) と 線分の方向ベクトル (lx, ly) の外積的なもの (2Dではスカラー)
                  cross_z = (cx - p1x)*ly - (cy - p1y)*lx

                  if cross_z > 1e-6: # 円が線分の右側にある場合 (P1からP2へのベクトルを見て Pyxel座標系)
                      nx = -ly / (mag_l if mag_l > 1e-6 else 1.0) # (-ly, lx) 向き
                      ny = lx / (mag_l if mag_l > 1e-6 else 1.0)
                  elif cross_z < -1e-6: # 円が線分の左側にある場合
                       nx = ly / (mag_l if mag_l > 1e-6 else 1.0) # (ly, -lx) 向き
                       ny = -lx / (mag_l if mag_l > 1e-6 else 0.0)
                  else: # cross_z がほぼ0の場合（線上に近い）
                      # 例えばデフォルト方向にするか、速度の向きで判断するなど
                      nx = perp_lx # 仮にperp方向にする
                      ny = perp_ly

                  # 再度正規化（mag_lが0の場合に備え）
                  mag_n = math.sqrt(nx*nx + ny*ny)
                  if mag_n > 1e-6: nx /= mag_n; ny /= mag_n
                  else: nx=1.0; ny=0.0 # デフォルト


        # めり込み解消 (衝突した分だけ法線方向に押し戻す)
        overlap = total_radius - dist_to_col
        if overlap > 1e-6: # めり込んでいる場合のみ押し出す (微小量を考慮)
            # print(f"  Overlap: {overlap:.4f}") # Debug
            # めり込み量を少し多めに押し出すことで貫通を防ぐ効果がある場合がある
            # 速度に応じて押し出し量を調整するとより物理的かもしれないが、ここでは固定値
            cx += nx * (overlap + 0.05) # ★少し多めに押し出す★ (0.05は調整値)
            cy += ny * (overlap + 0.05)
            # print(f"  Corrected pos: ({cx:.2f},{cy:.2f})") # Debug


        # 入射ベクトル (ボールの速度)
        vx_in = vx
        vy_in = vy

        # 速度の法線成分が正（ボールが線分から離れる方向）の場合は反射させない (貫通防止)
        # このチェックは、めり込み解消後の新しい位置と、調整前の速度ベクトルで行うのが一般的。
        # 法線ベクトルもめり込み解消後の位置から再計算するとより正確だが、簡易的に同じ法線を使う。
        dot_product_after_correction = vx_in * nx + vy_in * ny # 内積 (調整前の速度と法線で計算)
        if dot_product_after_correction > 0:
             # 離れる方向だが、めり込んでいる可能性があるので位置は調整する？
             # Simple Collision: 反射させず、そのままの速度で返す (位置調整は行った)
             # print("  Already separating, no reflection.") # Debug
             return True, vx, vy, cx, cy # 衝突はしたが反射はしない (位置調整は行った)


        # 反射計算 (R = I - 2(I . N)N) + フリッパー速度加算
        # dot_product は入射速度と法線の内積を使う
        new_vx = (vx_in - 2 * dot_product_after_correction * nx) * bounce_factor + flipper_v_at_collision_x # 反射・減衰 + フリッパー速度加算
        new_vy = (vy_in - 2 * dot_product_after_correction * ny) * bounce_factor + flipper_v_at_collision_y # 反射・減衰 + flipper速度加算

        # print(f"  Collision! Old V=({vx:.2f},{vy:.2f}), New V=({new_vx:.2f},{new_vy:.2f})") # Debug

        return True, new_vx, new_vy, cx, cy # 衝突した場合、新しい速度と位置を返す

    return False, vx, vy, cx, cy # 衝突しない (速度も位置もそのまま返す)

# Helper function for circle-circle collision and response
# 円と円の衝突判定と応答のためのヘルパー関数
# c1x, c1y, r1: 円1の中心座標と半径 (float)
# c2x, c2y, r2: 円2の中心座標と半径 (float)
# v1x, v1y: 円1の速度ベクトル (float)
# bounce_factor: 反発係数
# 戻り値: 衝突した場合 (True, 円1の新しい速度ベクトル, 円1の新しい位置), 衝突しない場合 (False, 円1の元の速度ベクトル, 円1の元の位置)
# ここでは円1(ボール)が動き、円2(バンパー)は静止している前提
def collide_circle_circle(c1x, c1y, r1, c2x, c2y, r2, v1x, v1y, bounce_factor):
    # 中心間のベクトル
    dx = c2x - c1x
    dy = c2y - c1x
    dist_sq = dx*dx + dy*dy # ★ここが間違っていました★ c1y と c2y を使うべき

    # 正しい中心間の距離の2乗
    dx = c2x - c1x
    dy = c2y - c1y
    dist_sq = dx*dx + dy*dy


    # 半径の合計
    total_radius = r1 + r2
    total_radius_sq = total_radius * total_radius

    # 距離が半径の合計以下であれば衝突
    if dist_sq <= total_radius_sq:
        # 衝突応答
        dist = math.sqrt(dist_sq) # dist_sq >= 0 は保証されているはず

        # 法線ベクトル (円2の中心から円1の中心へ向かうベクトルを正規化)
        # 円1から円2へ向かう方向 (dx, dy) の逆方向が法線
        if dist > 1e-6: # 中心が重なっていない場合
            nx = dx / dist # バンパー中心からボール中心へ
            ny = dy / dist
        else: # 中心が重なっている場合 (ゼロ除算対策)
             # 適当な法線ベクトルを設定 (例: 上方向)
             nx = 0.0
             ny = -1.0

        # めり込み解消 (衝突した分だけ法線方向に押し戻す)
        overlap = total_radius - dist
        if overlap > 1e-6: # めり込んでいる場合のみ押し出す
             # めり込み量を少し多めに押し出す
             push_dist = overlap + 0.05 # 0.05は調整値
             c1x -= nx * push_dist # 円1(ボール)を円2から離れる方向に押し戻す (nx, nyは円2から円1へのベクトルなので、引く)
             c1y -= ny * push_dist

        # 入射ベクトル (円1の速度)
        v1x_in = v1x
        v1y_in = v1y

        # 速度の法線成分が正（円1が円2から離れる方向）の場合は反射させない (貫通防止)
        # 法線ベクトルは円2から円1へ向かう方向 (nx, ny)
        # 速度ベクトルが法線方向を向いているか内積でチェック (v . n > 0 なら離れる方向)
        dot_product = v1x_in * nx + v1y_in * ny # 内積
        if dot_product < 0: # 速度が法線と逆方向（円1が円2に近づいている）場合のみ反射
             # 反射計算 (R = I - 2(I . N)N)
             # 円2は静止している前提なので、円1の速度のみ計算
             new_v1x = (v1x_in - 2 * dot_product * nx) * bounce_factor
             new_v1y = (v1y_in - 2 * dot_product * ny) * bounce_factor

             return True, new_v1x, new_v1y, c1x, c1y # 衝突した場合、新しい速度と位置を返す
        else:
             # 既に離れる方向、またはめり込み解消で離れた場合
             # 衝突はしたが反射はしない (位置調整は行った)
             return True, v1x, v1y, c1x, c1y # 衝突はしたが反射なし

    return False, v1x, v1y, c1x, c1y # 衝突しない (速度も位置もそのまま返す)


class Pinball:
    def __init__(self):
        # --- ウィンドウ設定 (initはメインガードで呼ぶ) ---
        self.WIDTH = 160
        self.HEIGHT = 240
        # pyxel.init(self.WIDTH, self.HEIGHT) はメインガードへ

        # --- ゲーム全体の状態 ---
        self.game_state = "READY" # "READY", "PLAYING", "GAME_OVER"
        self.balls = 3
        self.score = 0
        self.game_timer = 0 # ゲームが開始してからのフレーム数（演出などに使える）

        # --- ボールの状態 ---
        self.ball_x = 0.0 # 位置は浮動小数点数で持つ方が正確
        self.ball_y = 0.0
        self.ball_vx = 0.0
        self.ball_vy = 0.0
        self.ball_r = 3.0        # 半径もfloatに
        self.ball_color = 7      # 色 (白)

        # --- 物理定数 ---
        self.gravity = 0.1       # 重力加速度 (Y方向)
        self.friction = 0.99     # 空気抵抗や摩擦 (速度にかけることで減衰)
        self.bounce_factor = 0.8 # 反発係数 (壁など)
        # ボールの最大速度
        self.max_ball_speed = 15.0 # 例として15.0に設定（調整してください）
        # ボールの最低速度を設定
        self.min_ball_speed = 1.0 # 例として1.0に設定（調整してください）


        # --- プランジャーの状態 ---
        self.plunger_pull_time = 0 # 引いているフレーム数
        self.max_plunger_force = 8.0 # 最大打ち出し速度
        self.plunger_lane_x = self.WIDTH // 2 - 5 # レーン左端X
        self.plunger_lane_w = 10                 # レーン幅
        self.plunger_base_y = self.HEIGHT - 50   # プランジャー最下部Y

        self.plunger_max_pull_len = 10           # 引ける最大ピクセル数 (描画用)
        self.plunger_force_scale = 0.3           # 引く時間から速度への変換スケール
        self.plunger_side_force_scale = 0.5      # 横方向の速度に変換する割合
        # プランジャー右斜め発射時のランダム角度範囲
        self.plunger_random_angle_min_deg = 30.0 # 最小角度 (度)
        self.plunger_random_angle_max_deg = 60.0 # 最大角度 (度)


        # --- テーブルの構造 (描画関数で直接描く前提) ---
        self.wall_color = 13      # 壁の色 (薄緑)
        self.wall_thickness = 4   # 壁の厚さ
        # アウトレーンのY座標
        self.out_y_threshold = self.HEIGHT - 10 # 例として画面下から10ピクセル上の位置に設定

        # --- フリッパーの状態 ---
        self.flipper_len = 80.0 # フリッパーの長さ
        self.flipper_width = 6.0 # フリッパーの幅
        self.flipper_angle_min_deg = -30.0 # 下げた角度 (度)
        self.flipper_angle_max_deg = 30.0  # 上げた角度 (度)
        self.flipper_speed_deg = 8.0       # 毎フレームの回転速度 (度)
        self.flipper_bounce_factor = 2.0   # フリッパーの反発係数（壁より大きく）
        self.flipper_boost_speed_scale = 1.5 # フリッパーの速度加算の強さスケール

        # 左フリッパーの支点 (例: 画面下部の左右端から少し外側)
        self.flipper_l_pivot_x = float(self.wall_thickness - 10) # 左壁よりさらに左へ
        self.flipper_l_pivot_y = float(self.HEIGHT - 30)         # 画面下端から30ピクセル上

        # 右フリッパーの支点 (例: 画面下部の左右端から少し外側)
        self.flipper_r_pivot_x = float(self.WIDTH - self.wall_thickness + 10) # 右壁よりさらに右へ
        self.flipper_r_pivot_y = float(self.HEIGHT - 30)         # 画面下端から30ピクセル上

        # 現在のフリッパーの角度 (度)
        self.flipper_angle_l_deg = self.flipper_angle_min_deg
        self.flipper_angle_r_deg = self.flipper_angle_min_deg

        # 前のフレームのフリッパーの角度 (速度計算用)
        self.flipper_angle_l_prev_deg = self.flipper_angle_min_deg
        self.flipper_angle_r_prev_deg = self.flipper_angle_min_deg

        # --- 簡易的な貫通対策用の設定 ---
        self.sub_steps = 10 # 物理計算のサブステップ数（フレームを分割して計算する回数）

        # --- バンパーの状態 ---
        # バンパーのリスト: {"cx": float, "cy": float, "r": float, "score": int, "hit_timer": int}
        # バンパーを5つにしました (位置と数は前回と同じ)
        self.bumpers = [
            {"cx": self.WIDTH // 2,         "cy": 50,  "r": 8.0, "score": 100, "hit_timer": 0},
            {"cx": self.WIDTH // 2 - 30,    "cy": 90,  "r": 8.0, "score": 100, "hit_timer": 0}, # 位置調整
            {"cx": self.WIDTH // 2 + 30,    "cy": 90,  "r": 8.0, "score": 100, "hit_timer": 0}, # 位置調整
            {"cx": self.WIDTH // 2 - 15,    "cy": 130, "r": 8.0, "score": 100, "hit_timer": 0}, # 位置調整
            {"cx": self.WIDTH // 2 + 15,    "cy": 130, "r": 8.0, "score": 100, "hit_timer": 0}, # 位置調整
        ]
        self.bumper_hit_duration = 10 # バンパーが光るフレーム数
        self.bumper_color_normal = 8  # バンパーの色 (オレンジ)
        self.bumper_color_hit = 9     # ヒット時のバンパーの色 (茶色)
        # ★ここを元に戻す★ バンパーの反発係数を元に戻す (これは collide_circle_circle内で使われるが、その結果は後で上書き)
        self.bumper_bounce_factor = 5.0 # 例として5.0に設定（調整してください）


        # ゲームを初期状態にリセット
        self.reset_game()

    # --- reset_game メソッドは Pinball クラスのメソッドとして定義されているはずです ---
    def reset_game(self):
        """ゲームの状態を初期値にリセットする"""
        self.game_state = "READY"
        self.balls = 3
        self.score = 0
        self.plunger_pull_time = 0
        self.game_timer = 0
        # reset_ball_position はここで呼び出されます
        self.reset_ball_position() # ボールの位置と速度もリセット

        # フリッパーの角度もリセット
        self.flipper_angle_l_deg = self.flipper_angle_min_deg
        self.flipper_angle_r_deg = self.flipper_angle_min_deg
        self.flipper_angle_l_prev_deg = self.flipper_angle_min_deg
        self.flipper_angle_r_prev_deg = self.flipper_angle_min_deg

        # バンパーのヒット状態をリセット
        for bumper in self.bumpers:
             bumper["hit_timer"] = 0


    # --- reset_ball_position メソッドは Pinball クラスのメソッドとして定義されているはずです ---
    def reset_ball_position(self):
        """ボールを初期位置（プランジャー位置）に戻す"""
        self.ball_x = float(self.plunger_lane_x + self.plunger_lane_w // 2)
        self.ball_y = float(self.plunger_base_y + self.ball_r)
        self.ball_vx = 0.0
        self.ball_vy = 0.0


    def update(self):
        """ゲームの状態を毎フレーム更新する"""
        self.game_timer += 1 # ゲームタイマーを進める

        # --- フリッパーの角度更新 (キー入力に基づいて毎フレーム行う) ---
        # 左フリッパー (Zキー)
        target_angle_l = self.flipper_angle_min_deg
        if pyxel.btn(pyxel.KEY_Z):
            target_angle_l = self.flipper_angle_max_deg

        # 角度を滑らかに変化させる
        if self.flipper_angle_l_deg < target_angle_l:
             self.flipper_angle_l_deg = min(self.flipper_angle_l_deg + self.flipper_speed_deg, target_angle_l)
        elif self.flipper_angle_l_deg > target_angle_l:
             self.flipper_angle_l_deg = max(self.flipper_angle_l_deg - self.flipper_speed_deg, target_angle_l)


        # 右フリッパー (SLASHキー)
        target_angle_r = self.flipper_angle_min_deg
        if pyxel.btn(pyxel.KEY_SLASH):
             target_angle_r = self.flipper_angle_max_deg

        # 角度を滑らかに変化させる
        if self.flipper_angle_r_deg < target_angle_r:
             self.flipper_angle_r_deg = min(self.flipper_angle_r_deg + self.flipper_speed_deg, target_angle_r)
        elif self.flipper_angle_r_deg > target_angle_r:
             self.flipper_angle_r_deg = max(self.flipper_angle_r_deg - self.flipper_speed_deg, target_angle_r)

        # 前のフレームのフリッパー角度を保存 (物理計算の角速度計算用)
        # 角度更新の後に保存する
        self.flipper_angle_l_prev_deg = self.flipper_angle_l_deg
        self.flipper_angle_r_prev_deg = self.flipper_angle_r_deg

        # バンパーのヒット演出タイマーを減らす
        for bumper in self.bumpers:
            if bumper["hit_timer"] > 0:
                bumper["hit_timer"] -= 1


        # ゲーム状態による更新処理の振り分け
        if self.game_state == "READY":
            self.update_ready()
        elif self.game_state == "PLAYING":
            # 物理計算を複数のサブステップに分割して実行
            for _ in range(self.sub_steps):
                 # update_physics内でボールアウトするとREADY状態になるため、
                 # READY状態になったらループを中断するチェックを追加
                 if self.game_state != "PLAYING":
                      break # ボールアウトしたらサブステップを中断
                 self.update_physics(1.0 / self.sub_steps) # 1フレームの時間 (1.0) をサブステップ数で割った時間


        elif self.game_state == "GAME_OVER":
             self.update_game_over()

        # どこでも共通のリトライ処理 (Rキー)
        if self.game_state == "GAME_OVER" and pyxel.btnp(pyxel.KEY_R):
            self.reset_game()

    def update_physics(self, dt):
        """物理計算と衝突判定を分割された時間 dt で実行"""

        # --- ボールの物理演算 ---
        # 重力と摩擦を微小時間 dt に応じて適用
        self.ball_vy += self.gravity * dt

        # 空気抵抗や摩擦による速度の減衰 (速度にfriction^(dt) をかける)
        friction_dt = self.friction**(dt)
        self.ball_vx *= friction_dt
        self.ball_vy *= friction_dt

        # 速度に基づいてボールの位置を更新 (微小時間 dt で)
        self.ball_x += self.ball_vx * dt
        self.ball_y += self.ball_vy * dt

        # --- 速度制限 ---
        speed = math.sqrt(self.ball_vx**2 + self.ball_vy**2)
        if speed > self.max_ball_speed:
             scale = self.max_ball_speed / speed
             self.ball_vx *= scale
             self.ball_vy *= scale
        # 最低速度の適用
        elif speed > 1e-6 and speed < self.min_ball_speed: # 速度がほぼゼロではなく、最低速度より小さい場合
             scale = self.min_ball_speed / speed
             self.ball_vx *= scale
             self.ball_vy *= scale


        # --- 衝突判定と応答 ---
        # 壁との衝突 (ボールの位置がめり込んでいたら調整)
        # ササブステップごとに判定・調整
        # 左壁
        if self.ball_x - self.ball_r < self.wall_thickness:
            self.ball_x = self.wall_thickness + self.ball_r # 壁の境界まで位置を戻す
            self.ball_vx *= -self.bounce_factor # X速度を反転・減衰
            # print(f"Wall L Collision! New pos=({self.ball_x:.2f}, {self.ball_y:.2f}) v=({self.ball_vx:.2f}, {self.ball_vy:.2f})")

        # 右壁
        if self.ball_x + self.ball_r > self.WIDTH - self.wall_thickness:
            self.ball_x = self.WIDTH - self.wall_thickness - self.ball_r
            self.ball_vx *= -self.bounce_factor
            # print(f"Wall R Collision! New pos=({self.ball_x:.2f}, {self.ball_y:.2f}) v=({self.ball_vx:.2f}, {self.ball_vy:.2f})")

        # 上壁
        if self.ball_y - self.ball_r < self.wall_thickness:
             self.ball_y = self.wall_thickness + self.ball_r
             self.ball_vy *= -self.bounce_factor
             # print(f"Wall U Collision! New pos=({self.ball_x:.2f}, {self.ball_y:.2f}) v=({self.ball_vx:.2f}, {self.ball_vy:.2f})")


        # 下壁 (アウトレーン)
        # アウト判定はサブステップごとに判定
        if self.ball_y + self.ball_r > self.out_y_threshold:
             # print("Ball Out! -> lose_ball()")
             self.lose_ball()
             # ボールアウトしたら、このサブステップの物理処理はここで終了
             return # サブステップ関数から抜ける

        # --- フリッパーとの衝突 ---
        # フリッパーの現在の角度と前のフレームの角度から角速度（速度）を計算 (フレーム単位で計算された値を使用)
        # update() で計算・保存された角度 (self.flipper_angle_l_deg, self.flipper_angle_l_prev_deg など) を使用する。
        # 毎フレームの角度変化量 (度/フレーム)
        # 左フリッパー: 下げた角度 min_deg から 上げた角度 max_deg へ (角度の値が増える方向)
        angular_change_l_deg_frame = self.flipper_angle_l_deg - self.flipper_angle_l_prev_deg
        # 右フリッパー: 下げた角度 min_deg から 上げた角度 max_deg へ (角度の値が増える方向)
        angular_change_r_deg_frame = self.flipper_angle_r_deg - self.flipper_angle_r_prev_deg

        # サブステップごとの角速度 (ラジアン/サブステップ)
        # 左フリッパー: 物理的な角速度 = Pyxel基準角度変化量 * (-1) / sub_steps
        angular_velocity_l_physical_dt = math.radians(-angular_change_l_deg_frame) / self.sub_steps

        # 右フリッパー: 物理的な角速度 = -(Pyxel基準角度変化量) / sub_steps
        angular_velocity_r_physical_dt = math.radians(-angular_change_r_deg_frame) / self.sub_steps


        # 左フリッパーとの衝突判定 (当たり判定は中心線分に対して行う)
        # 衝突点におけるフリッパーの速度を計算して collide_line_circle に渡す
        # ボールの現在位置から支点へのベクトル (相対座標)
        r_ball_lx = self.ball_x - self.flipper_l_pivot_x
        r_ball_ly = self.ball_y - self.flipper_l_pivot_y
        # 衝突点でのフリッパー速度 v = omega x r (2D: vx = -omega * ry, vy = omega * rx)
        # 左フリッパーの物理的な角速度 angular_velocity_l_physical_dt を使用
        flipper_v_at_ball_vx_l_dt = -angular_velocity_l_physical_dt * r_ball_ly
        flipper_v_at_ball_vy_l_dt = angular_velocity_l_physical_dt * r_ball_lx


        # 左フリッパー線分の端点座標を計算 (現在の角度)
        angle_l_rad_pyxel_current = math.radians(-self.flipper_angle_l_deg)
        fl_tip_x = self.flipper_l_pivot_x + self.flipper_len * math.cos(angle_l_rad_pyxel_current)
        fl_tip_y = self.flipper_l_pivot_y + self.flipper_len * math.sin(angle_l_rad_pyxel_current)


        collided_l, self.ball_vx, self.ball_vy, self.ball_x, self.ball_y = collide_line_circle(
            self.flipper_l_pivot_x, self.flipper_l_pivot_y,
            fl_tip_x, fl_tip_y, # フリッパー先端座標 (現在の角度)
            self.ball_x, self.ball_y, self.ball_r,
            self.flipper_width / 2.0, # 当たり判定にフリッパーの太さの半分を加算
            self.ball_vx, self.ball_vy, self.flipper_bounce_factor, # フリッパー用反発係数
            flipper_v_at_ball_vx_l_dt * self.flipper_boost_speed_scale, flipper_v_at_ball_vy_l_dt * self.flipper_boost_speed_scale # フリッパーの速度加算 (サブステップごと)
        )


        # 右フリッパーとの衝突判定 (左と衝突しなかった場合)
        # ボールの現在位置から支点へのベクトル (相対座標) - 左フリッパーとの衝突で位置が変わっている可能性があるので、新しい位置を使う
        r_ball_rx = self.ball_x - self.flipper_r_pivot_x
        r_ball_ry = self.ball_y - self.flipper_r_pivot_y

        # 衝突点でのフリッパー速度 v = omega x r (2D: vx = -omega * ry, vy = omega * rx)
        # 右フリッパーの物理的な角速度 angular_velocity_r_physical_dt を使用
        # 右フリッパーの速度加算計算 (簡易方式: 左フリッパーの速度加算を左右反転して使う簡易方式)
        flipper_v_at_ball_vx_r_dt = -flipper_v_at_ball_vx_l_dt # X方向反転
        flipper_v_at_ball_vy_r_dt = flipper_v_at_ball_vy_l_dt # Y方向同じ

        # 右フリッパー線分の端点座標を計算 (現在の角度)
        angle_r_rad_pyxel_current = math.radians(180 + self.flipper_angle_r_deg)
        fr_tip_x = self.flipper_r_pivot_x + self.flipper_len * math.cos(angle_r_rad_pyxel_current)
        fr_tip_y = self.flipper_r_pivot_y + self.flipper_len * math.sin(angle_r_rad_pyxel_current)


        # 左フリッパーと衝突しなかった場合のみ右フリッパーと衝突判定
        if not collided_l:
            collided_r, self.ball_vx, self.ball_vy, self.ball_x, self.ball_y = collide_line_circle(
                self.flipper_r_pivot_x, self.flipper_r_pivot_y,
                fr_tip_x, fr_tip_y, # フリッパー先端座標 (現在の角度)
                self.ball_x, self.ball_y, self.ball_r,
                 self.flipper_width / 2.0, # 当たり判定にフリッパーの太さの半分を加算
                self.ball_vx, self.ball_vy, self.flipper_bounce_factor, # フリッパー用反発係数
                flipper_v_at_ball_vx_r_dt * self.flipper_boost_speed_scale, flipper_v_at_ball_vy_r_dt * self.flipper_boost_speed_scale # フリッパーの速度加算 (サブステップごと)
            )

        # --- バンパーとの衝突判定 ---
        # バンパーもサブステップごとに判定・処理
        # フリッパーとの衝突でボールの位置や速度が変わっている可能性があるので、最新の値を使う
        for bumper in self.bumpers:
             # collide_circle_circle は位置と速度を更新したタプルを返す
             # Note: collide_circle_circle は速度を反射計算しますが、
             # ここではその結果は使わず、速度の向きをランダムに設定します。
             collided_bumper, _, _, temp_x, temp_y = collide_circle_circle( # 一時変数で受け取る (速度は使わないので _ で受ける)
                 self.ball_x, self.ball_y, self.ball_r, # ボールの情報 (位置は衝突で変わっている可能性があるので最新を使う)
                 bumper["cx"], bumper["cy"], bumper["r"], # バンパーの情報
                 self.ball_vx, self.ball_vy, # ボールの速度 (collide_circle_circle内での反射計算には使われるが、その結果は破棄)
                 self.bumper_bounce_factor # バンパー用反発係数 (collide_circle_circle内で使われるが、その結果は破棄)
             )
             if collided_bumper:
                 # バンパーとの衝突があった場合
                 # collide_circle_circleでめり込み解消された位置は反映する
                 self.ball_x = temp_x
                 self.ball_y = temp_y

                 # ★ここを変更★ ボールの速度の向きをランダムな角度(90～270度)に設定
                 # 衝突前の速度の大きさを取得
                 current_speed = math.sqrt(self.ball_vx**2 + self.ball_vy**2)
                 # 速度の大きさが非常に小さい場合は最低速度に設定
                 if current_speed < self.min_ball_speed:
                     current_speed = self.min_ball_speed

                 # 90度から270度の範囲でランダムな角度を生成 (度数)
                 # Pyxel座標系ではY下向きが正。90度は真下、180度は真左、270度は真上、0/360度は真右。
                 # 90度から270度なので、真下から真上までの左半円になります。
                 random_angle_deg = random.uniform(90.0, 270.0) # 度数でランダムな角度
                 random_angle_rad = math.radians(random_angle_deg) # ラジアンに変換

                 # 新しい速度ベクトルを計算 (大きさは衝突前のスピード、向きはランダムな角度)
                 self.ball_vx = current_speed * math.cos(random_angle_rad)
                 self.ball_vy = current_speed * math.sin(random_angle_rad)

                 # バンパーに当たったらスコア加算
                 self.score += bumper["score"]
                 # バンパーのヒット演出タイマーを設定
                 bumper["hit_timer"] = self.bumper_hit_duration
                 # バンパーのヒット音を鳴らす (TODO)
                 # pyxel.play(0, 0) # サウンド番号などを指定


    def update_ready(self):
        """ゲーム開始前の待機状態（プランジャー操作）の更新処理"""

        # プランジャー操作 (スペースキー)
        if pyxel.btn(pyxel.KEY_SPACE):
            max_pull_frames = int(self.max_plunger_force / self.plunger_force_scale) + 30
            self.plunger_pull_time = min(self.plunger_pull_time + 1, max_pull_frames)

        elif pyxel.btnr(pyxel.KEY_SPACE):
            # 引いていた時間に応じて基本的な速度（縦方向）を計算
            base_plunger_force = min(self.plunger_pull_time * self.plunger_force_scale, self.max_plunger_force)

            if self.plunger_pull_time > 0:
                 # 発射時の速度を斜めにする
                 # 左右キーが押されているかチェック
                 move_left = pyxel.btn(pyxel.KEY_LEFT)
                 move_right = pyxel.btn(pyxel.KEY_RIGHT)

                 if move_left and not move_right: # 左キーが押されている
                     self.ball_vx = -base_plunger_force * self.plunger_side_force_scale # 左方向速度
                     self.ball_vy = -base_plunger_force * (1.0 - self.plunger_side_force_scale) # 上方向速度 (少し減らす)
                 elif move_right and not move_left: # 右キーが押されている
                     # 右キーと同時にスペースキーを離した場合、ランダムな角度で発射
                     # 速度の大きさは base_plunger_force になるように調整
                     speed_magnitude = base_plunger_force # 速度の大きさ
                     # ランダムな角度を度数で生成 (30度から60度の範囲)
                     # Pyxel座標系での角度は -60度から-30度 (真上から右に30度～60度)
                     random_angle_deg_pyxel = random.uniform(-60.0, -30.0)
                     random_angle_rad_pyxel = math.radians(random_angle_deg_pyxel)

                     # 新しい速度ベクトルを計算
                     self.ball_vx = speed_magnitude * math.cos(random_angle_rad_pyxel)
                     self.ball_vy = speed_magnitude * math.sin(random_angle_rad_pyxel)


                 else: # 左右キーが押されていないか、両方押されている
                     self.ball_vx = 0.0 # 横方向速度なし
                     self.ball_vy = -base_plunger_force # 真上方向速度 (負の値で上向き)

                 self.game_state = "PLAYING"

            self.plunger_pull_time = 0

        # READY状態では、ボールの位置をプランジャーに固定
        plunger_offset_y = min(self.plunger_pull_time * 0.2, self.plunger_max_pull_len)
        self.ball_x = float(self.plunger_lane_x + self.plunger_lane_w // 2)
        self.ball_y = float(self.plunger_base_y + self.ball_r - plunger_offset_y)


    def update_game_over(self):
        """ゲームオーバー状態の更新処理"""
        pass


    def lose_ball(self):
        """ボールを失う処理"""
        self.balls -= 1
        if self.balls <= 0:
            self.game_state = "GAME_OVER"
            self.game_timer = 0
        else:
            self.game_state = "READY"
            self.reset_ball_position() # ボールの初期位置に戻す
            self.plunger_pull_time = 0


    def draw(self):
        """ゲーム画面を毎フレーム描画する"""
        pyxel.cls(0)

        # --- テーブルの壁を描画 ---
        pyxel.rect(0, 0, self.wall_thickness, self.HEIGHT, self.wall_color) # 左壁
        pyxel.rect(self.WIDTH - self.wall_thickness, 0, self.wall_thickness, self.HEIGHT, self.wall_color) # 右壁
        pyxel.rect(0, 0, self.WIDTH, self.wall_thickness, self.wall_color) # 上壁

        # アウトレーンの境界線
        pyxel.line(0, self.out_y_threshold, self.WIDTH, self.out_y_threshold, 8) # 赤い線

        # --- プランジャーと射出レーンを描画 ---
        # プランジャー射出レーン全体を薄緑色で塗りつぶす
        pyxel.rect(self.plunger_lane_x, self.wall_thickness, self.plunger_lane_w, self.HEIGHT - self.wall_thickness, self.wall_color)


        # READY状態ならプランジャーバーを描画
        if self.game_state == "READY":
             plunger_ui_h = min(self.plunger_pull_time * 0.2, self.plunger_max_pull_len)
             bar_x = self.plunger_lane_x + self.plunger_lane_w // 2
             bar_top_y = self.plunger_base_y + self.ball_r - plunger_ui_h
             pyxel.rect(bar_x - 2, bar_top_y, 4, plunger_ui_h, 8) # 赤色のバー


        # --- フリッパーを描画 ---
        flipper_color = 10 # 明るい緑色

        # 左フリッパー
        # 角度をラジアンに変換 (Pyxel基準角度)
        angle_l_rad_pyxel = math.radians(-self.flipper_angle_l_deg)

        # 支点からの相対座標 (長さ方向に沿ったものと、幅方向に沿ったもの)
        # 根元 (支点から幅方向に ±flipper_width/2)
        base_offset_x_l_plus = (self.flipper_width / 2) * math.cos(angle_l_rad_pyxel + math.pi/2) # 角度 + 90度 (幅方向)
        base_offset_y_l_plus = (self.flipper_width / 2) * math.sin(angle_l_rad_pyxel + math.pi/2)
        base_offset_x_l_minus = (self.flipper_width / 2) * math.cos(angle_l_rad_pyxel - math.pi/2) # 角度 - 90度 (幅方向)
        base_offset_y_l_minus = (self.flipper_width / 2) * math.sin(angle_l_rad_pyxel - math.pi/2)

        # 先端 (支点から長さ方向に flipper_len, 幅方向に ±flipper_width/2)
        tip_offset_x_l = self.flipper_len * math.cos(angle_l_rad_pyxel)
        tip_offset_y_l = self.flipper_len * math.sin(angle_l_rad_pyxel)
        tip_offset_x_l_plus = tip_offset_x_l + (self.flipper_width / 2) * math.cos(angle_l_rad_pyxel + math.pi/2)
        tip_offset_y_l_plus = tip_offset_y_l + (self.flipper_width / 2) * math.sin(angle_l_rad_pyxel + math.pi/2)
        tip_offset_x_l_minus = tip_offset_x_l + (self.flipper_width / 2) * math.cos(angle_l_rad_pyxel - math.pi/2)
        tip_offset_y_l_minus = tip_offset_y_l + (self.flipper_width / 2) * math.sin(angle_l_rad_pyxel - math.pi/2)

        # 矩形の4隅の絶対座標 (支点 + 相対座標)
        # 頂点 P1, P2 は根元、P3, P4 は先端
        p1x_l = self.flipper_l_pivot_x + base_offset_x_l_plus
        p1y_l = self.flipper_l_pivot_y + base_offset_y_l_plus
        p2x_l = self.flipper_l_pivot_x + base_offset_x_l_minus
        p2y_l = self.flipper_l_pivot_y + base_offset_y_l_minus
        p3x_l = self.flipper_l_pivot_x + tip_offset_x_l_minus
        p3y_l = self.flipper_l_pivot_y + tip_offset_y_l_minus
        p4x_l = self.flipper_l_pivot_x + tip_offset_x_l_plus
        p4y_l = self.flipper_l_pivot_y + tip_offset_y_l_plus

        # 2つの三角形で矩形を描画 (P1-P2-P3 と P1-P3-P4)
        pyxel.tri(int(p1x_l), int(p1y_l), int(p2x_l), int(p2y_l), int(p3x_l), int(p3y_l), flipper_color)
        pyxel.tri(int(p1x_l), int(p1y_l), int(p3x_l), int(p3y_l), int(p4x_l), int(p4y_l), flipper_color)


        # 右フリッパー
        # 角度をラジアンに変換 (Pyxel基準角度)
        # 右フリッパーは 180度 + self.flipper_angle_r_deg が Pyxel基準角度
        angle_r_rad_pyxel = math.radians(180 + self.flipper_angle_r_deg)

        # 支点からの相対座標 (長さ方向に沿ったものと、幅方向に沿ったもの)
        # 根元 (支点から幅方向に ±flipper_width/2)
        base_offset_x_r_plus = (self.flipper_width / 2) * math.cos(angle_r_rad_pyxel + math.pi/2) # 角度 + 90度 (幅方向)
        base_offset_y_r_plus = (self.flipper_width / 2) * math.sin(angle_r_rad_pyxel + math.pi/2)
        base_offset_x_r_minus = (self.flipper_width / 2) * math.cos(angle_r_rad_pyxel - math.pi/2) # 角度 - 90度 (幅方向)
        base_offset_y_r_minus = (self.flipper_width / 2) * math.sin(angle_r_rad_pyxel - math.pi/2)

        # 先端 (支点から長さ方向に flipper_len, 幅方向に ±flipper_width/2)
        tip_offset_x_r = self.flipper_len * math.cos(angle_r_rad_pyxel)
        tip_offset_y_r = self.flipper_len * math.sin(angle_r_rad_pyxel)
        tip_offset_x_r_plus = tip_offset_x_r + (self.flipper_width / 2) * math.cos(angle_r_rad_pyxel + math.pi/2)
        tip_offset_y_r_plus = tip_offset_y_r + (self.flipper_width / 2) * math.sin(angle_r_rad_pyxel + math.pi/2)
        tip_offset_x_r_minus = tip_offset_x_r + (self.flipper_width / 2) * math.cos(angle_r_rad_pyxel - math.pi/2)
        tip_offset_y_r_minus = tip_offset_y_r + (self.flipper_width / 2) * math.sin(angle_r_rad_pyxel - math.pi/2)


        # 矩形の4隅の絶対座標 (支点 + 相対座標)
        # 頂点 P1, P2 は根元、P3, P4 は先端
        p1x_r = self.flipper_r_pivot_x + base_offset_x_r_plus
        p1y_r = self.flipper_r_pivot_y + base_offset_y_r_plus
        p2x_r = self.flipper_r_pivot_x + base_offset_x_r_minus
        p2y_r = self.flipper_r_pivot_y + base_offset_y_r_minus
        p3x_r = self.flipper_r_pivot_x + tip_offset_x_r_minus
        p3y_r = self.flipper_r_pivot_y + tip_offset_y_r_minus
        p4x_r = self.flipper_r_pivot_x + tip_offset_x_r_plus
        p4y_r = self.flipper_r_pivot_y + tip_offset_y_r_plus


        # 2つの三角形で矩形を描画 (P1-P2-P3 と P1-P3-P4)
        pyxel.tri(int(p1x_r), int(p1y_r), int(p2x_r), int(p2y_r), int(p3x_r), int(p3y_r), flipper_color)
        pyxel.tri(int(p1x_r), int(p1y_r), int(p3x_r), int(p3y_r), int(p4x_r), int(p4y_r), flipper_color)


        # 支点の円を描画 (任意 - フリッパーが回転しているように見せるため)
        # 円の中心は支点、半径はフリッパー幅の半分より少し大きくすると見た目が良いかも
        pivot_circle_r = int(self.flipper_width / 2) + 1
        pyxel.circ(int(self.flipper_l_pivot_x), int(self.flipper_l_pivot_y), pivot_circle_r, flipper_color)
        pyxel.circ(int(self.flipper_r_pivot_x), int(self.flipper_r_pivot_y), pivot_circle_r, flipper_color)

        # --- バンパーを描画 ---
        for bumper in self.bumpers:
            # ヒットしている場合は色を変える
            color = self.bumper_color_hit if bumper["hit_timer"] > 0 else self.bumper_color_normal
            # 円として描画 (中心x, 中心y, 半径, 色)
            pyxel.circ(int(bumper["cx"]), int(bumper["cy"]), int(bumper["r"]), color)


        # 3. ボールを描画
        if self.game_state != "GAME_OVER":
             # ボールの位置は浮動小数点数だが、描画は整数座標で行う
             pyxel.circ(int(self.ball_x), int(self.ball_y), int(self.ball_r), self.ball_color) # 半径もintに

        # 4. UI (ユーザーインターフェース) を描画
        pyxel.text(self.wall_thickness + 5, self.wall_thickness + 5, f"SCORE: {self.score}", 7)
        pyxel.text(self.wall_thickness + 5, self.wall_thickness + 15, f"BALLS: {self.balls}", 7)

        if self.game_state == "READY":
             launch_text = "PRESS SPACE TO LAUNCH"
             launch_text_width = len(launch_text) * 4
             pyxel.text(self.WIDTH//2 - launch_text_width // 2, self.HEIGHT - 60, launch_text, 7)

        elif self.game_state == "GAME_OVER":
            game_over_text = "GAME OVER"
            game_over_width = len(game_over_text) * 4
            pyxel.text(self.WIDTH//2 - game_over_width // 2, self.HEIGHT//2, game_over_text, 8)

            retry_text = "PRESS R TO RETRY"
            retry_width = len(retry_text) * 4
            pyxel.text(self.WIDTH//2 - retry_width // 2, self.HEIGHT//2 + 10, retry_text, 7)


# --- ゲームの開始 ---
if __name__ == "__main__":
    pyxel.init(160, 240)

    game = Pinball()

    pyxel.run(game.update, game.draw)