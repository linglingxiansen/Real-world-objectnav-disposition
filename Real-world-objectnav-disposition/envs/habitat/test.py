import pyzed.sl as sl
import cv2 as cv
def main():
    zed = sl.Camera()
    InitPara = sl.InitParameters()
    InitPara.camera_fps = 30    
    InitPara.camera_resolution = sl.RESOLUTION.HD720
    err = zed.open(InitPara)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(-1)
    left_img = sl.Mat()
    depth = sl.Mat()
    runtimeParas = sl.RuntimeParameters()    
    while(True):    
        if zed.grab(runtimeParas) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(left_img, view=sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            depth_image = depth.get_data()
            time_stamp = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)            
            print(time_stamp.get_milliseconds())            
            img = left_img.get_data()      
            print(img.shape)      
            cv.imshow("img", img)  
            cv.imshow("depth", depth_image)         
            cv.waitKey(5)
if __name__ == "__main__":    main()