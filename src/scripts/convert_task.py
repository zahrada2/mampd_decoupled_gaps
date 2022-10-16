import xml.etree.ElementTree as ET
import sys

# Convert MovingAI type task file to TSP task file.
# use: python convert_task.py input_path.xmp output_path.tsp

if len(sys.argv) != 3:
    print("   Please specify input and output files. Example use:")
    print("       python convert_task.py input_path.xmp output_path.tsp")
    sys.exit(0)

infile = sys.argv[1]
root = ET.parse(infile).getroot()
f = open(sys.argv[2], "w")

name = infile.split('.')[0]
f.write("NAME: {}".format(name) + '\n')
f.write("TYPE: TSP" + '\n')
f.write("COMMENT: Parsed Moving AI benchmark task file." + '\n')

dim = len(root.findall('agents/agent')) + len(root.findall('targets/target'))
f.write("DIMENSION: {}".format(dim) + '\n')
f.write("EDGE_WEIGHT_TYPE: EUC_2D" + '\n')
f.write("NODE_COORD_SECTION" + '\n')

id = 1

for type_tag in root.findall('agents/agent'):
    aid = type_tag.get('id')
    start_x = type_tag.get('start.i')
    start_y = type_tag.get('start.j')
    goal_x = type_tag.get('goal.i')
    goal_y = type_tag.get('goal.j')
    
    f.write(str(int(id)) + ' ' + str(float(start_x)) + ' ' + str(float(start_y)) + '\n')
    id = id + 1

for type_tag in root.findall('targets/target'):
    nid = type_tag.get('id')
    x = type_tag.get('i')
    y = type_tag.get('j')
    
    f.write(str(int(id)) + ' ' + str(float(x)) + ' ' + str(float(y)) + '\n')
    id = id + 1

f.write("EOF")
f.close()
