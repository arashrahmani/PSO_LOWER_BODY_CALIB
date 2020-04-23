import json

# open output file for reading
q = []
s = []
with open('r_leg_data.txt', 'r') as filehandle:
    r_logs_list = json.loads(json.dumps([q, s]))
    print("right logs list: ",r_logs_list)