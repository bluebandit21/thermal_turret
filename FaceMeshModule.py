
import cv2
import mediapipe as mp
import time
import math

class FaceMeshDetector():

    def __init__(self, staticMode=False, maxFaces=2, minDetectionCon=0.5, minTrackCon=0.5):

        self.staticMode = staticMode
        self.maxFaces = maxFaces
        self.minDetectionCon = minDetectionCon
        self.minTrackCon = minTrackCon

        self.mpDraw = mp.solutions.drawing_utils
        self.mpFaceMesh = mp.solutions.face_mesh
        self.faceMesh = self.mpFaceMesh.FaceMesh(self.staticMode, self.maxFaces,
                                                 self.minDetectionCon, self.minTrackCon)
        self.drawSpec = self.mpDraw.DrawingSpec(thickness=1, circle_radius=2)

    def findFaceMesh(self, img, draw=True):
        self.imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.faceMesh.process(self.imgRGB)
        faces = []
        my_dict = {"103": [0,0], "10": [0,0], "9": [0,0],"284": [0,0]};
        if self.results.multi_face_landmarks:

            for faceLms in self.results.multi_face_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, faceLms, self.mpFaceMesh.FACE_CONNECTIONS,
                                               self.drawSpec, self.drawSpec)
                face = []
                #ids = [67,69,109,108,151,10,338,332,333,299,337,284,298,293,103,104]
                ids = [103,10,9,284]
                for id, lm in enumerate(faceLms.landmark):
                    # print(lm)
                    ih, iw, ic = img.shape
                    x, y = int(lm.x * iw), int(lm.y * ih)
                    if id in ids:
                        z = str(x) + ',' + str(y)
                        cv2.putText(img, z, (x, y), cv2.FONT_HERSHEY_PLAIN,
                                1, (0, 255, 0), 1)

                    if id in ids:
                        my_dict[str(id)] = [x,y]
                    face.append([x,y])
                faces.append(face)




        return img, faces, my_dict


def main():
    cap = cv2.VideoCapture("Videos/1.mp4")
    pTime = 0
    detector = FaceMeshDetector()
    while True:
        success, img = cap.read()
        img,faces, my_dict = detector.findFaceMesh(img,False)


        center = (math.ceil(img.shape[1]/2)-1,math.ceil(img.shape[0]/2)-1)
        print (center)
        print(my_dict)
        if (my_dict["103"][0] < center[0]) and (my_dict["10"][1] < center[1]) and (my_dict["9"][1] > center[1]) and (my_dict["284"][0] > center[0]):
            cv2.rectangle(img, (math.ceil(img.shape[1] / 2) - 40, math.ceil(img.shape[0] / 2) - 40),
                          (math.ceil(img.shape[1] / 2) + 40, math.ceil(img.shape[0] / 2) + 40), (255, 0, 0), 3)
            if len(faces) != 0:
                print(my_dict)

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        cv2.putText(img, f'FPS: {int(fps)}', (20, 70), cv2.FONT_HERSHEY_PLAIN,
                    3, (0, 255, 0), 3)
        #center dot
        cv2.rectangle(img, (math.ceil(img.shape[1]/2)-1,math.ceil(img.shape[0]/2)-1), (math.ceil(img.shape[1]/2)+1,math.ceil(img.shape[0]/2)+1), (255,0,0), -1)
        cv2.imshow("Image", img)
        cv2.waitKey(1)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()