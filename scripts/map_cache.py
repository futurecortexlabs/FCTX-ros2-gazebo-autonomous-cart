"""地図の時刻だけが変わった場合の再計算を避ける、内容の完全一致キー。"""
def occupancy_signature(msg):
    info=msg.info;p=info.origin.position;q=info.origin.orientation
    data=msg.data.tobytes() if hasattr(msg.data,'tobytes') else bytes((v & 255 for v in msg.data))
    # ハッシュ値ではなく実データを比較し、衝突による地図更新の見落としを避ける。
    return (msg.header.frame_id,info.width,info.height,info.resolution,p.x,p.y,p.z,q.x,q.y,q.z,q.w,data)
