code1 - database.py:
    system argument 1: input file name
    system argument 2: output file name
    example: working_directory\data\database\Circle.bvh working_directory\output\database\Circle.py

code2 - manual_locator.py:
    system argument 1: input file directory
    system argument 2: output file name
    example: working_directory\data\manual\Boxing working_directory\data\manual\Boxing.txt
    
    Input file directory should contain two folders named front and side, containing all front-view keyframes and side-view keyframes respectively.
    Front-view keyframes should be named front (1 + interval * n).jpg
    Side-view keyframes should be named side (1 + interval * n).jpg

code3 - manual_reader.py:
    system argument 1: input file name
    system argument 2: output file name
    example: working_directory\data\manual\Moonwalk.txt working_directory\output\manual\manual-Moonwalk.py
    
    Input file format can be seen in working_directory\data\manual\Moonwalk.txt

code4 - cv_reader.py:
    system argument 1: input file name (front view)
    system argument 2: input file name (side view)
    system argument 3: output file name
    example: working_directory\data\cv\Boxing-front.txt working_directory\data\cv\Boxing-side.txt working_directory\output\cv\cv-boxing.py

    Input files format can be seen in working_directory\data\cv\Boxing-front.txt and working_directory\data\cv\Boxing-side.txt

OpenPose indeices of joints:
00 head
01 upperchest
02 right upper arm
03 right lower arm
04 right hand
05 left upper arm
06 left lower arm
07 left hand
08 hips
09 right upper leg
10 right lower leg
11 right ankle
12 left upper leg
13 left lower leg
14 left ankle
15 right eye
16 left eye
17 right ear
18 left ear
19 left toes
20 left toes 2
21 left heel
22 right toes
23 right toes 2
24 right heel

cv-reader indices of vectors:
00 hips position
01 right upper leg -> left upper leg
02 right upper leg -> right lower leg
03 right lower leg -> right heel
04 right heel -> right toes
05 left upper leg -> left lower leg
06 left lower leg -> left heel
07 left heel -> left toes
08 right upper arm -> left upper arm
09 right upper arm -> right lower arm
10 right lower arm -> right hand
11 left upper arm -> left lower arm
12 left lower arm -> left hand

manual reader indices of joints:
00 hips
01 right upper leg
02 right lower leg
03 right foot
04 right toes
05 left upper leg
06 left lower leg
07 left foot
08 left toes
09 spine
10 chest
11 upper chest
12 right shoulder
13 right upper arm
14 right lower arm
15 right hand
16 left shoulder
17 left upper arm
18 left lower arm
19 left hand
20 neck
21 head
22 jaw

manual-reader indices of vectors:
00 hips position
01 right upper leg -> left upper leg
02 right upper leg -> right lower leg
03 right lower leg -> right foot
04 right foot -> right toes
05 left upper leg -> left lower leg
06 left lower leg -> left foot
07 left foot -> left toes
08 spine -> chest
09 chest -> upper chest
10 right shoulder -> left shoulder
11 right shoulder -> right upper arm
12 right upper arm -> right lower arm
13 right lower arm -> right hand
14 left shoulder -> left upper arm
15 left upper arm -> left lower arm
16 left lower arm -> left hand
17 neck -> head
18 head -> jaw
