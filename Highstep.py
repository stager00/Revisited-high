from picrawler import Picrawler
from robot_hat import TTS, Music
from robot_hat import Ultrasonic
from robot_hat import Pin
import time
import cv2
import readchar
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont

# Initialize OLED display
RST = None
disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST)
disp.begin()
disp.clear()
disp.display()

# Create blank image for drawing.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))
draw = ImageDraw.Draw(image)
font = ImageFont.load_default()

tts = TTS()
music = Music()

crawler = Picrawler() 
sonar = Ultrasonic(Pin("D2"), Pin("D3"))
music.music_set_volume(100)

alert_distance = 15  # Distance threshold for detecting obstacles
view_block_threshold = 0.5  # Threshold for deciding if the view is blocked (50% of the view)
speed = 80

manual = '''
Press keys on keyboard to control PiCrawler!
    W: Forward
    A: Turn left
    S: Backward
    D: Turn right
    0: Switch to obstacle avoidance mode
    1: Toggle obstacle detection
    2: Double height and step height
    3: Speed 70%
    4: Speed 100%
    5: Speed 150%
    C: Toggle camera feed
    U: Look up
    J: Look down

    Ctrl^C: Quit
'''

def show_info():
    print("\033[H\033[J", end='')  # clear terminal window
    print(manual)

def get_ultrasonic_distance():
    distance = sonar.read()
    return distance

def is_view_blocked():
    # Capture image from the camera
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    cap.release()

    if not ret:
        print("Failed to capture image")
        return False

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply edge detection
    edges = cv2.Canny(gray, 50, 150)

    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        print("No contours found")
        return False

    # Assume the largest contour is the obstacle
    largest_contour = max(contours, key=cv2.contourArea)

    # Get the bounding box of the largest contour
    x, y, w, h = cv2.boundingRect(largest_contour)

    # Calculate the proportion of the view blocked by the obstacle
    frame_height, frame_width = gray.shape
    blocked_proportion = h / frame_height

    return blocked_proportion > view_block_threshold

def high_step():
    # Define higher step coordinates
    steps = [
        [[60, 0, -20], [60, 0, -20], [60, 0, -20], [60, 0, -20]],  # Initial position with higher step
        [[70, 10, -10], [50, -10, -30], [70, 10, -10], [50, -10, -30]],  # Move legs 1 and 3 with higher step
        [[60, 0, -20], [60, 0, -20], [60, 0, -20], [60, 0, -20]],  # Back to initial position with higher step
        [[50, -10, -30], [70, 10, -10], [50, -10, -30], [70, 10, -10]],  # Move legs 2 and 4 with higher step
        [[60, 0, -20], [60, 0, -20], [60, 0, -20], [60, 0, -20]]   # Back to initial position with higher step
    ]
    for step in steps:
        crawler.do_step(step, speed=speed)

def normal_step():
    # Define normal step coordinates
    steps = [
        [[60, 0, -30], [60, 0, -30], [60, 0, -30], [60, 0, -30]],  # Initial position
        [[70, 10, -20], [50, -10, -40], [70, 10, -20], [50, -10, -40]],  # Move legs 1 and 3
        [[60, 0, -30], [60, 0, -30], [60, 0, -30], [60, 0, -30]],  # Back to initial position
        [[50, -10, -40], [70, 10, -20], [50, -10, -40], [70, 10, -20]],  # Move legs 2 and 4
        [[60, 0, -30], [60, 0, -30], [60, 0, -30], [60, 0, -30]]   # Back to initial position
    ]
    for step in steps:
        crawler.do_step(step, speed=speed)

def double_height_step():
    # Define double height and step coordinates
    steps = [
        [[60, 0, -60], [60, 0, -60], [60, 0, -60], [60, 0, -60]],  # Initial position with double height
        [[70, 10, -40], [50, -10, -80], [70, 10, -40], [50, -10, -80]],  # Move legs 1 and 3 with double step height
        [[60, 0, -60], [60, 0, -60], [60, 0, -60], [60, 0, -60]],  # Back to initial position with double height
        [[50, -10, -80], [70, 10, -40], [50, -10, -80], [70, 10, -40]],  # Move legs 2 and 4 with double step height
        [[60, 0, -60], [60, 0, -60], [60, 0, -60], [60, 0, -60]]   # Back to initial position with double height
    ]
    for step in steps:
        crawler.do_step(step, speed=speed)

def look_up():
    # Define coordinates to look up (lower rear, raise front)
    steps = [
        [[60, 0, -30], [60, 0, -30], [60, 0, -10], [60, 0, -10]],  # Lower rear, raise front
        [[60, 0, -30], [60, 0, -30], [60, 0, -10], [60, 0, -10]]   # Maintain position
    ]
    for step in steps:
        crawler.do_step(step, speed=speed)

def look_down():
    # Define coordinates to look down (raise rear, lower front)
    steps = [
        [[60, 0, -10], [60, 0, -10], [60, 0, -30], [60, 0, -30]],  # Raise rear, lower front
        [[60, 0, -10], [60, 0, -10], [60, 0, -30], [60, 0, -30]]   # Maintain position
    ]
    for step in steps:
        crawler.do_step(step, speed=speed)

def obstacle_avoidance_mode():
    distance = get_ultrasonic_distance()
    print(f"Distance: {distance} cm")
    
    # Display distance on OLED
    draw.rectangle((0, 0, width, height), outline=0, fill=0)
    draw.text((0, 0), f"Distance: {distance} cm", font=font, fill=255)
    if distance <= alert_distance:
        draw.text((0, 20), "Obstacle detected!", font=font, fill=255)
    disp.image(image)
    disp.display()
    
    if distance < 0:
        pass
    elif distance <= alert_distance:
        if is_view_blocked():
            try:
                music.sound_play_threading('./sounds/sign.wav', volume=100)
            except Exception as e:
                print(e)
            print("Turning left to avoid the obstacle")
            crawler.do_action('turn left angle', 3, speed)
            time.sleep(0.2)
        else:
            print("Stepping over the obstacle")
            high_step()  # Call the high step function
    else:
        print("Path clear. Normal
