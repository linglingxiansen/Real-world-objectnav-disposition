import pyzed.sl as sl
import numpy as np
import cv2
image = sl.Mat()
depth = sl.Mat()
def _preprocess_depth(depth, min_d=0.5, max_d=5):
    """预处理深度图像
    Args:
        depth: 输入的深度图像
        min_d: 最小深度值(米)
        max_d: 最大深度值(米)
    Returns:
        处理后的深度图像
    """
    # 处理超出范围的深度值
    
    invalid = np.isnan(depth) | np.isinf(depth)

    depth[invalid] = 0

    mask_far = depth > max_d
    depth[mask_far] = max_d
    
    mask_near = depth < min_d
    depth[mask_near] = min_d
    print(np.min(depth),np.max(depth))
    # 归一化到0-1范围
    depth = (depth - min_d) / (max_d - min_d)
    
    # 处理特殊值
    mask2 = depth > 0.99
    depth[mask2] = 0.0
    
    mask1 = depth == 0
    depth[mask1] = 100.0
    
    # 缩放到合适的范围
    depth = min_d * 100.0 + depth * (max_d - min_d) * 100.0
    
    return depth

class ZED2Camera:
    def __init__(self, width=1280, height=720, fps=30):
        # self.width = width
        # self.height = height
        self.fps = fps
        
        # 初始化相机
        self.zed = sl.Camera()
        
        # 创建初始化参数
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720  # 可根据需要调整分辨率
        self.init_params.camera_fps = fps
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # 可根据需要调整深度模式
        self.init_params.coordinate_units = sl.UNIT.METER  # 使用米作为单位
        self.init_params.depth_minimum_distance = 0.5
        self.init_params.depth_maximum_distance = 5 
        # 创建运行时参数
        self.runtime_params = sl.RuntimeParameters()
        # self.runtime_params.confidence_threshold = 50
        # self.runtime_params.texture_confidence_threshold = 100
        self.depth_mat = sl.Mat()
        self.rgb_mat = sl.Mat()
        self.is_running = False

    def start(self):
        """启动相机"""
        if not self.is_running:
            try:
                err = self.zed.open(self.init_params)
                if err != sl.ERROR_CODE.SUCCESS:
                    print(f"打开相机失败: {err}")
                    return False
                    
                self.is_running = True
                print("等待相机预热...")
                # 预热相机
                for _ in range(30):
                    if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                        continue
                print("相机已准备就绪")
                return True
                
            except Exception as e:
                print(f"启动相机失败: {str(e)}")
                return False
                
        return True

    def stop(self):
        """停止相机"""
        if self.is_running:
            self.zed.close()
            self.is_running = False

    def get_frame(self,image,depth):
        """获取单帧RGBD图像
        Returns:
            color_image: RGB图像 (numpy array)
            depth_image: 深度图像 (numpy array)
            depth_colormap: 深度图的彩色可视化 (numpy array)
        """
        if not self.is_running:
            print("相机未启动，请先启动相机。")
            return None, None, None
        
        try:
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                # 准备图像容器
                
                
                # 获取左目图像（RGB）
                self.zed.retrieve_image(image, view=sl.VIEW.LEFT)
                color_image = image.get_data()
                # print(color_image.shape)

                
                # 获取深度图
                self.zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                depth_image = depth.get_data()
                # color_image = np.asanyarray(color_image)
                depth_image = np.asanyarray(depth_image)
                # print(depth_image)
                # exit()
                # print(np.min(depth_image),np.max(depth_image))
                # 创建深度图的彩色可视化
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), 
                    cv2.COLORMAP_JET
                )
                depth_image = np.expand_dims(depth_image, axis=-1)
                return color_image[:,:,:3], depth_image, depth_colormap
                
            return None, None, None
            
        except Exception as e:
            print(f"获取帧失败: {str(e)}")
            return None, None, None

    def __del__(self):
        """清理资源"""
        self.stop()

# 使用示例
if __name__ == "__main__":
    camera = ZED2Camera()
    
    if camera.start():
        try:
            while True:
                color_img, depth_img, depth_colormap = camera.get_frame(image,depth)
                
                if color_img is not None:
                    # 处理深度图像
                    processed_depth = _preprocess_depth(depth_img.copy())
                    print(np.min(processed_depth),np.max(processed_depth))
                    # 显示图像
                    cv2.imshow("Color Image", color_img)
                    cv2.imwrite("Depth.png", processed_depth)
                    cv2.imshow("Processed Depth", depth_colormap)
                    cv2.waitKey(5)
                    # 按'q'退出
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                        
        finally:
            camera.stop()
            cv2.destroyAllWindows()
