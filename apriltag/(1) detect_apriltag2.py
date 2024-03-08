# AprilTag 인식 기본 코드
# AprilTag 라이브러리 설치 필요

import apriltag
import cv2

img = cv2.imread("C:\Users\msi\Desktop\apriltag_IMG",cv2.IMREAD_COLOR)

class Test:
    def April(self):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        detector = apriltag.Detector()
        result = detector.detect(gray)
        
        print(result)
        
        for r in result:
            for idx in range(len(r.corners)):
                cv2.line(img, tuple(r.corners[idx-1, :].astype(int)), tuple(r.corners[idx, :].astype(int)), (0, 255, 0), 3)
            
            cv2.putText(img, str(r.tag_id), org=(r.corners[0,0].astype(int)+10, r.corners[0, 1].astype(int)+10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0, 0, 255), thickness=2)           
            
            tagFamily = r.tag_family.decode("utf-8")
            print(r.center)
        
        
        cv2.imshow("test", img)
        
        print("family is: "+tagFamily)
        
        if cv2.waitKey(0) == 27:
            cv2.destroyAllWindows()


if __name__=="__main__":
    t=Test()
    t.April()

