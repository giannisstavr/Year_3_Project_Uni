import cv2
import numpy as np
import pygame
import serial
import time
from serial import Serial, SerialException


# Initialize pygame
pygame.init()

# Define variables
selected_points = []
object_image = None
coords = []
coords1 = []
coords2 = []
out_intercept = 0
count = 0
counter = 0
tracking = False
main_output_sent = False

# Define a callback function that will be called when the user clicks on the image
def select_object(event, x, y, flags, param):
    global selected_points, object_image, tracking  # Add tracking to the global variables
    if event == cv2.EVENT_LBUTTONDOWN:
        selected_points = [(x, y)]
    elif event == cv2.EVENT_MOUSEMOVE:
        if flags & cv2.EVENT_FLAG_LBUTTON:
            if len(selected_points) > 0:
                temp = frame.copy()
                cv2.rectangle(temp, selected_points[0], (x, y), (0, 255, 0), 2)
                cv2.imshow("Camera", temp)
    elif event == cv2.EVENT_LBUTTONUP:
        selected_points.append((x, y))
        cv2.rectangle(frame, selected_points[0], selected_points[1], (0, 255, 0), 2)
        cv2.imshow("Camera", frame)
        if len(selected_points) > 1:
            x1, y1 = selected_points[0]
            x2, y2 = selected_points[1]
            object_image = frame[y1:y2, x1:x2]
            if object_image is not None and object_image.shape[:2] > (0,0):
                tracker.init(frame, (x1, y1, x2-x1, y2-y1))
            tracking = True  # Set tracking to True when the left mouse button is released


# Open the camera
cap = cv2.VideoCapture(0)

tracker = cv2.legacy.TrackerMOSSE_create()

#different trackers for testng 
#tracker = cv2.legacy.TrackerCSRT_create()
#tracker = cv2.legacy.TrackerTLD_create()
#tracker = cv2.legacy.TrackerMedianFlow_create()import cv2
#tracker = cv2.legacy.TrackerBoosting_create()
#tracker = cv2.legacy.TrackerMIL_create()
#tracker = cv2.legacy.TrackerKCF_create()


# Continuously capture frames from the camera and display them
while count < 600:
    # Read a frame from the camera
    _, frame = cap.read()
    if tracking:
        success, bbox = tracker.update(frame)
        
        if success:
            x, y, w, h = [int(v) for v in bbox]
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, "Object tracked", (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            object_image = frame[y:y+h, x:x+w]
            
            if object_image is not None and object_image.shape[0] > 0 and object_image.shape[1] > 0:

                if success:
                    x, y, w, h = [int(v) for v in bbox]
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

                if count == 0:
                    coords.append((x,y))
                    count += 1
                else:
                    prev_x, prev_y = coords[-1]
                    if (x != prev_x) or (y != prev_y):
                        coords.append((x, y))
                        count += 1
                        if count % 15 == 0:
                            ncoords = coords[-1:]
                            # Receive data on location (input)
                            coords1.append(float(ncoords[0][0]))
                            coords2.append(float(ncoords[0][1]))

                            if len(coords1) and len(coords2) >= 3:
                                v = np.array(coords1)
                                c = np.array(coords2)

                                #get line of best fit
                                a, b = np.polyfit(v, c, 1)

                                #camera_height=79cm
                                gline= 600
                                
                                intercept=gline*a+b
                                out_intercept= 0.0998*intercept-20.03
                                print(out_intercept)
                                #print(intercept)
                                counter==counter+1
                                
                                if __name__ == '__main__':
                                    counter = 1
                                    while True:
                                        if counter == 1:
                                            try:
                                                ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
                                                ser.reset_input_buffer()
                                                counter += 1
                                            except serial.SerialException as e:
                                                print(f"Error opening serial port: {e}")
                                                time.sleep(1)
                                                continue
                                                
                                        try:
                                            ser.write(str(out_intercept).encode() + b"\n")
                                            line = ser.readline().decode('utf-8').rstrip()
                                            print(line)
                                            time.sleep(1)
                                        except serial.SerialException as e:
                                            print(f"Serial communication error: {e}")
                                            time.sleep(1)
                                            continue
                                
                                
                              
                        if out_intercept != 0:
                            break
            else:
                cv2.putText(frame, "Object not tracked", (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                break
    else:
        cv2.putText(frame, "Select object to track", (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("Camera", frame)

    # Handle user input
    cv2.setMouseCallback("Camera", select_object)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
pygame.quit()
cap.release()
cv2.destroyAllWindows()
