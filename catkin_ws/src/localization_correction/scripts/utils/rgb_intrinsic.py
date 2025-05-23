import pyrealsense2 as rs

# 파이프라인 구성
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # RGB 스트림 활성화

# 파이프라인 시작
profile = pipeline.start(config)

# 스트림에서 RGB 카메라 프로필 얻기
color_stream = profile.get_stream(rs.stream.color)
intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

# Intrinsic 정보 출력
print(f"Width: {intrinsics.width}")
print(f"Height: {intrinsics.height}")
print(f"Fx: {intrinsics.fx}")
print(f"Fy: {intrinsics.fy}")
print(f"Cx: {intrinsics.ppx}")
print(f"Cy: {intrinsics.ppy}")
print(f"Distortion Model: {intrinsics.model}")
print(f"Distortion Coefficients: {intrinsics.coeffs}")

# 종료
pipeline.stop()

