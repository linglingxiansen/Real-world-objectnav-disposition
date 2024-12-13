import pyrealsense2 as rs
import numpy as np
import cv2
def _preprocess_depth(depth, min_d=0, max_d=10000):
        for i in range(depth.shape[1]):
            col_data = depth[:, i]
            # Only consider non-zero values for max
            valid_depths = col_data[col_data > 0]
            if valid_depths.size > 0:
                max_val = valid_depths.max()
                depth[:, i][col_data == 0] = max_val
        
        print(np.min(depth),np.max(depth))
        # Remove points beyond max_depth (similar to original's 0.99 threshold)
        mask_far = depth > max_d
        depth[mask_far] = 0
        
        mask_near = depth < min_d
        depth[mask_near] = 0.0
        
        # Set remaining zeros to a large value (similar to original's 100.0)
        mask_zero = depth == 0
        depth[mask_zero] = 100.0
        
        # Scale depth to match the original processing
        # Map depth values from [min_depth, max_depth] to similar range as original
        normalized_depth = (depth - min_d) / (max_d-min_d)
        scaled_depth = min_d /10 + normalized_depth * (max_d-min_d) /10
        
        return scaled_depth
    
def _preprocess_depth1(depth, min_d=0.5, max_d=5):
    
        # depth  = depth/1000
        # for i in range(depth.shape[1]):
        #     col_data = depth[:, i]
        #     # Only consider non-zero values for max
        #     valid_depths = col_data[col_data > 0]
        #     if valid_depths.size > 0:
        #         max_val = valid_depths.max()
        #         depth[:, i][col_data == 0] = max_val
        # 0-10
        print(np.min(depth),np.max(depth))
        # Remove points beyond max_depth (similar to original's 0.99 threshold)
        mask_far = depth > max_d
        depth[mask_far] = max_d
        
        mask_near = depth < min_d
        depth[mask_near] = min_d
        
        depth = (depth - min_d) / (max_d-min_d)
        
        # 0-1
        print(np.min(depth),np.max(depth))
        mask2 = depth > 0.99
        depth[mask2] = 0.
        # print(np.min(depth),np.max(depth))
        mask1 = depth == 0
        depth[mask1] = 100.0
        # print(np.min(depth),np.max(depth))
        # exit()
        # depth = min_d * 100.0 + depth * max_d * 100.0
        depth = min_d * 100.0 + depth * (max_d-min_d) * 100.0
        # Set remaining zeros to a large value (similar to original's 100.0)
        # mask_zero = depth == 0
        # depth[mask_zero] = 100.0
        
        # Scale depth to match the original processing
        # Map depth values from [min_depth, max_depth] to similar range as original
        
        # scaled_depth = min_d *100 + depth * (max_d-min_d) *100
        
        return depth
class RealSenseCamera:
    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        
        # 初始化相机
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # 配置图像流
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        
        # 创建对齐对象
        self.align = rs.align(rs.stream.color)
        self.is_running = False

    def start(self):
        """启动相机"""
        if not self.is_running:
            try:
                self.pipeline.start(self.config)
                self.is_running = True
                # 预热相机
                print("等待相机预热...")
                for _ in range(30):
                    self.pipeline.wait_for_frames()
                print("相机已准备就绪")
            except Exception as e:
                print(f"启动相机失败: {str(e)}")
    
    def stop(self):
        """停止相机"""
        if self.is_running:
            self.pipeline.stop()
            self.is_running = False
        
    def get_frame(self):
        """获取单帧对齐的RGBD图像
        返回值:
            color_image: RGB图像 (numpy array)
            depth_image: 深度图像 (numpy array)
            depth_colormap: 深度图的彩色可视化 (numpy array)
        """
        if not self.is_running:
            print("相机未启动，请先启动相机。")
            return None, None, None
        
        try:
            # 获取帧
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            
            # 获取对齐后的帧
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                print("未能获取完整的帧")
                return None, None, None
                
            # 转换为numpy数组
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data()) * 0.0010000000474974513
            
            # exit()
            # depth_image = np.asanyarray(depth_frame.get_data())
            # print(np.min(depth_image),np.max(depth_image))
            
            # 创建深度图像的彩色映射
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            
            return color_image, depth_image, depth_colormap
            
        except Exception as e:
            print(f"获取帧失败: {str(e)}")
            return None, None, None
            
    def __del__(self):
        """清理资源"""
        self.stop()

# 使用示例
if __name__ == "__main__":
    camera = RealSenseCamera()
    camera.start()
    
    try:
        while True:
            color_img, depth_img, depth_colormap = camera.get_frame()
            depth = _preprocess_depth1(depth_img)
            print(np.min(depth),np.max(depth))
            cv2.imwrite('./depth.png',depth)
            exit()
            # print(np.min(depth),np.max(depth))
            if color_img is not None:
                cv2.imshow("Color Image", color_img)
                cv2.imshow("Depth Colormap", depth_colormap)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        camera.stop()
        cv2.destroyAllWindows()