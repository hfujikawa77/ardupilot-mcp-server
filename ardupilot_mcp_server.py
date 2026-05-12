from mcp.server.fastmcp import FastMCP
from pymavlink import mavutil
import math
import time

# MCP初期化
mcp = FastMCP("ArduPilot Controller", debug=True)
print("MCPサーバー初期化完了")

ARMABLE_MODES = {
    "ACRO",
    "ALT_HOLD",
    "BRAKE",
    "DRIFT",
    "GUIDED",
    "LOITER",
    "POSHOLD",
    "SPORT",
    "STABILIZE",
}

MISSION_TYPE = getattr(mavutil.mavlink, "MAV_MISSION_TYPE_MISSION", 0)

# ArduPilot接続（ローカルMAVProxy UDPリレー）
def connect_to_ardupilot():
    mavlink_endpoint = "127.0.0.1:14550"
    try:
        print(f"ArduPilotに接続中... ({mavlink_endpoint})")
        conn = mavutil.mavlink_connection(
            mavlink_endpoint,
            source_system=1,
            source_component=90,
            autoreconnect=True
        )
        print("接続オブジェクト作成完了、ハートビート待機中...")
        if not conn.wait_heartbeat(timeout=10):
            raise Exception("10秒間ハートビートを受信できませんでした")
        print(f"ArduPilotに接続しました (endpoint: {mavlink_endpoint}, システムID: {conn.target_system}, コンポーネントID: {conn.target_component})")
        return conn
    except Exception as e:
        print(f"接続エラー: {str(e)}")
        print("接続設定を確認してください:")
        print("- SITL/実機が起動しているか")
        print(f"- MAVLinkエンドポイントが正しいか ({mavlink_endpoint})")
        print("- MAVProxyリレーが動作しているか")
        print("- ファイアウォール設定")
        raise

def ensure_armable_mode(conn) -> str | None:
    current_mode = conn.flightmode
    if current_mode in ARMABLE_MODES:
        return None

    mode_id = conn.mode_mapping().get("GUIDED")
    if mode_id is None:
        return "エラー: GUIDEDモードが利用できません"

    conn.set_mode(mode_id)
    start_time = time.time()
    while time.time() - start_time < 5:
        if conn.flightmode == "GUIDED":
            return f"アーム可能なモードではないため、{current_mode} から GUIDED に変更しました。"
        time.sleep(0.1)

    return f"エラー: GUIDEDモードへの変更を確認できませんでした (現在のモード: {conn.flightmode})"

def get_current_position(conn) -> dict:
    msg = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=10)
    if not msg:
        raise Exception("現在位置を取得できませんでした")

    return {
        "lat": msg.lat / 1e7,
        "lon": msg.lon / 1e7,
        "relative_alt_m": msg.relative_alt / 1000.0,
        "alt_m": msg.alt / 1000.0,
        "heading_deg": msg.hdg / 100.0 if msg.hdg != 65535 else None,
    }

def offset_lat_lon(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    lat_rad = math.radians(lat)
    new_lat = lat + north_m / 111320.0
    new_lon = lon + east_m / (111320.0 * math.cos(lat_rad))
    return new_lat, new_lon

def build_star_waypoints(
    center_lat: float,
    center_lon: float,
    altitude: float,
    outer_radius: float,
    inner_radius: float,
    points: int,
    yaw_offset_deg: float,
    close_shape: bool,
) -> list[dict]:
    waypoints = []
    vertex_count = points * 2
    for index in range(vertex_count):
        radius = outer_radius if index % 2 == 0 else inner_radius
        angle = math.radians(yaw_offset_deg + (360.0 * index / vertex_count))
        north_m = radius * math.cos(angle)
        east_m = radius * math.sin(angle)
        lat, lon = offset_lat_lon(center_lat, center_lon, north_m, east_m)
        waypoints.append({"lat": lat, "lon": lon, "alt": altitude})

    if close_shape and waypoints:
        waypoints.append(waypoints[0].copy())

    return waypoints

def mission_item(seq: int, command: int, lat: float, lon: float, alt: float, current: int = 0) -> dict:
    return {
        "seq": seq,
        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        "command": command,
        "current": current,
        "autocontinue": 1,
        "param1": 0,
        "param2": 0,
        "param3": 0,
        "param4": 0,
        "x": int(lat * 1e7),
        "y": int(lon * 1e7),
        "z": alt,
    }

def send_mission_item(conn, item: dict, use_int: bool = True):
    if not use_int:
        conn.mav.mission_item_send(
            conn.target_system,
            conn.target_component,
            item["seq"],
            item["frame"],
            item["command"],
            item["current"],
            item["autocontinue"],
            item["param1"],
            item["param2"],
            item["param3"],
            item["param4"],
            item["x"] / 1e7,
            item["y"] / 1e7,
            item["z"],
        )
        return

    conn.mav.mission_item_int_send(
        conn.target_system,
        conn.target_component,
        item["seq"],
        item["frame"],
        item["command"],
        item["current"],
        item["autocontinue"],
        item["param1"],
        item["param2"],
        item["param3"],
        item["param4"],
        item["x"],
        item["y"],
        item["z"],
    )

def upload_mission_items(conn, items: list[dict]):
    conn.mav.mission_count_send(
        conn.target_system,
        conn.target_component,
        len(items),
        MISSION_TYPE,
    )

    sent = set()
    deadline = time.time() + 30
    while time.time() < deadline:
        msg = conn.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=1)
        if not msg:
            continue

        msg_type = msg.get_type()
        if msg_type == "MISSION_ACK":
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                return
            raise Exception(f"ミッションアップロードが拒否されました: ACK type={msg.type}")

        seq = msg.seq
        if seq < 0 or seq >= len(items):
            raise Exception(f"不正なミッション要求を受信しました: seq={seq}")

        send_mission_item(conn, items[seq], use_int=msg_type == "MISSION_REQUEST_INT")
        sent.add(seq)

    raise Exception(f"ミッションアップロードがタイムアウトしました (送信済み {len(sent)}/{len(items)})")

def request_mission_item(conn, seq: int):
    conn.mav.mission_request_int_send(
        conn.target_system,
        conn.target_component,
        seq,
        MISSION_TYPE,
    )
    msg = conn.recv_match(type=["MISSION_ITEM_INT", "MISSION_ITEM"], blocking=True, timeout=10)
    if not msg:
        raise Exception(f"ミッションアイテム {seq} を取得できませんでした")
    return msg

# アーム
@mcp.tool()
def arm() -> str:
    conn = connect_to_ardupilot()
    try:
        if not conn.wait_heartbeat(timeout=10):
            return "エラー: ArduPilotとの接続がタイムアウトしました"
        mode_message = ensure_armable_mode(conn)
        if mode_message and mode_message.startswith("エラー:"):
            return mode_message
        conn.arducopter_arm()
        conn.motors_armed_wait()
        if mode_message:
            return f"{mode_message}\n機体をアームしました。"
        return "機体をアームしました。"
    except Exception as e:
        return f"エラー: {str(e)}\n接続設定を確認してください:\n- SITL/実機が起動しているか\n- ポート番号が正しいか (5762)\n- ファイアウォール設定"
    finally:
        conn.close()

# ディスアーム
@mcp.tool()
def disarm() -> str:
    conn = connect_to_ardupilot()
    try:
        conn.wait_heartbeat()
        conn.arducopter_disarm()
        conn.motors_disarmed_wait()
        return "機体をディスアームしました。"
    finally:
        conn.close()

# 離陸（高度指定）
@mcp.tool()
def takeoff(altitude: float = 10.0) -> str:
    conn = connect_to_ardupilot()
    try:
        # ハートビート待機（タイムアウト10秒）
        if not conn.wait_heartbeat(timeout=10):
            return "エラー: ArduPilotとの接続がタイムアウトしました"

        # 現在のモードを確認
        current_mode = conn.flightmode
        if current_mode != "GUIDED":
            # GUIDEDモードに変更
            conn.set_mode(conn.mode_mapping().get("GUIDED"))
            time.sleep(1)

        # アーム処理
        conn.arducopter_arm()
        conn.motors_armed_wait()
        time.sleep(1)

        # 離陸コマンド送信
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,    # confirmation
            0,    # param1: Minimum pitch
            0,    # param2: Empty
            0,    # param3: Empty
            0,    # param4: Yaw angle
            0,    # param5: Latitude
            0,    # param6: Longitude
            altitude  # param7: Altitude
        )

        # コマンド受信確認
        msg = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
        if not msg or msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return "エラー: 離陸コマンドが拒否されました"

        return f"{altitude}m の高度まで離陸を開始しました。"
    finally:
        conn.close()

# モード変更
@mcp.tool()
def change_mode(mode: str) -> str:
    conn = connect_to_ardupilot()
    try:
        conn.wait_heartbeat()
        mode_id = conn.mode_mapping().get(mode.upper())
        if mode_id is None:
            return f"無効なモードです: {mode}"
        
        # モード変更コマンド送信
        conn.set_mode(mode_id)
        
        # モード変更確認 (最大5秒待機)
        start_time = time.time()
        while time.time() - start_time < 5:
            if conn.flightmode == mode.upper():
                return f"モードを {mode.upper()} に変更しました。"
            time.sleep(0.1)
        
        return f"警告: モード変更を確認できませんでした (現在のモード: {conn.flightmode})"
    finally:
        conn.close()

# ステータス確認
@mcp.tool()
def get_status() -> dict:
    conn = connect_to_ardupilot()
    try:
        conn.wait_heartbeat()
        heartbeat = conn.messages.get('HEARTBEAT')
        return {
            "armed": conn.motors_armed(),
            "mode": conn.flightmode,
            "system_status": heartbeat.system_status if heartbeat else None
        }
    finally:
        conn.close()

# 現在位置確認
@mcp.tool()
def get_position() -> dict:
    conn = connect_to_ardupilot()
    try:
        conn.wait_heartbeat()
        return get_current_position(conn)
    finally:
        conn.close()

# ミッション消去
@mcp.tool()
def clear_mission() -> str:
    conn = connect_to_ardupilot()
    try:
        conn.wait_heartbeat()
        conn.mav.mission_clear_all_send(
            conn.target_system,
            conn.target_component,
            MISSION_TYPE,
        )

        msg = conn.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
        if not msg:
            return "警告: ミッション消去ACKを確認できませんでした"
        if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            return f"エラー: ミッション消去が拒否されました: ACK type={msg.type}"
        return "ミッションを消去しました。"
    finally:
        conn.close()

# ミッション取得
@mcp.tool()
def download_mission() -> dict:
    conn = connect_to_ardupilot()
    try:
        conn.wait_heartbeat()
        conn.mav.mission_request_list_send(
            conn.target_system,
            conn.target_component,
            MISSION_TYPE,
        )
        count_msg = conn.recv_match(type="MISSION_COUNT", blocking=True, timeout=10)
        if not count_msg:
            return {"error": "ミッション件数を取得できませんでした"}

        items = []
        for seq in range(count_msg.count):
            msg = request_mission_item(conn, seq)
            lat = msg.x / 1e7
            lon = msg.y / 1e7
            if msg.get_type() == "MISSION_ITEM":
                lat = msg.x
                lon = msg.y

            items.append({
                "seq": msg.seq,
                "command": msg.command,
                "frame": msg.frame,
                "lat": lat,
                "lon": lon,
                "alt": msg.z,
            })

        return {"count": count_msg.count, "items": items}
    finally:
        conn.close()

# 星形ミッション作成・アップロード
@mcp.tool()
def upload_star_mission(
    altitude: float = 15.0,
    outer_radius: float = 50.0,
    inner_radius: float = 20.0,
    points: int = 5,
    center_lat: float = 0.0,
    center_lon: float = 0.0,
    yaw_offset_deg: float = 90.0,
    include_takeoff: bool = True,
    close_shape: bool = True,
    clear_existing: bool = True,
) -> dict:
    if points < 3:
        return {"error": "points は 3 以上を指定してください"}
    if altitude <= 0:
        return {"error": "altitude は 0 より大きい値を指定してください"}
    if outer_radius <= 0 or inner_radius <= 0:
        return {"error": "outer_radius と inner_radius は 0 より大きい値を指定してください"}
    if inner_radius >= outer_radius:
        return {"error": "inner_radius は outer_radius より小さい値を指定してください"}

    conn = connect_to_ardupilot()
    try:
        conn.wait_heartbeat()

        center_source = "specified"
        if center_lat == 0.0 and center_lon == 0.0:
            position = get_current_position(conn)
            center_lat = position["lat"]
            center_lon = position["lon"]
            center_source = "current_position"

        if clear_existing:
            conn.mav.mission_clear_all_send(
                conn.target_system,
                conn.target_component,
                MISSION_TYPE,
            )
            conn.recv_match(type="MISSION_ACK", blocking=True, timeout=5)

        star_waypoints = build_star_waypoints(
            center_lat,
            center_lon,
            altitude,
            outer_radius,
            inner_radius,
            points,
            yaw_offset_deg,
            close_shape,
        )

        mission_items = []
        if include_takeoff:
            mission_items.append(mission_item(
                0,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                center_lat,
                center_lon,
                altitude,
                current=1,
            ))

        for waypoint in star_waypoints:
            mission_items.append(mission_item(
                len(mission_items),
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                waypoint["lat"],
                waypoint["lon"],
                waypoint["alt"],
                current=0 if mission_items else 1,
            ))

        upload_mission_items(conn, mission_items)

        return {
            "message": "星形ミッションをアップロードしました。",
            "mission_count": len(mission_items),
            "center_source": center_source,
            "center_lat": center_lat,
            "center_lon": center_lon,
            "altitude": altitude,
            "outer_radius": outer_radius,
            "inner_radius": inner_radius,
            "points": points,
            "include_takeoff": include_takeoff,
            "close_shape": close_shape,
        }
    except Exception as e:
        return {"error": str(e)}
    finally:
        conn.close()

# ミッション開始
@mcp.tool()
def start_mission() -> str:
    conn = connect_to_ardupilot()
    try:
        conn.wait_heartbeat()
        mode_id = conn.mode_mapping().get("AUTO")
        if mode_id is None:
            return "エラー: AUTOモードが利用できません"

        conn.set_mode(mode_id)
        start_time = time.time()
        while time.time() - start_time < 5:
            if conn.flightmode == "AUTO":
                return "AUTOモードに変更し、ミッション開始状態にしました。"
            time.sleep(0.1)

        return f"警告: AUTOモードへの変更を確認できませんでした (現在のモード: {conn.flightmode})"
    finally:
        conn.close()

if __name__ == "__main__":
    print("MCPサーバーを起動します...")
    print(f"利用可能なツール: {[func.__name__ for func in [arm, disarm, takeoff, change_mode, get_status, get_position, clear_mission, download_mission, upload_star_mission, start_mission]]}")
    print("クライアントからの接続を待機中...")
    mcp.run(transport="stdio")
    print("MCPサーバーを終了します")
