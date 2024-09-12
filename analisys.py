import pandas as pd
import json


with open("offloaded_data.json") as f:
    data = json.load(f)

df_0 = pd.json_normalize(data)

#Option to analyze the first x entries, if x is greater than the maximum polling_sqn then all entries will be analyzed
max_req_polling_sqn = 1000
max_polling_sqn = df_0['polling_sqn'].max()-1
if (max_req_polling_sqn>max_polling_sqn):
    print_max = max_polling_sqn
else:
    print_max= max_req_polling_sqn     
print("\n***\n",print_max,"REQUESTS FROM THE BASE STATION WILL BE ANALYZED\n***\n")


df = df_0[(df_0['polling_sqn'] < print_max) & (df_0['polling_sqn'] > 0)]

#Counting numbers of cache misses
df_404 = df[df['data._data']=='404']           
grouped_404 = df_404.groupby('polling_sqn')['balloon_id'].count()
total_balloons = df['balloon_id'].nunique()
cache_miss_count = (grouped_404 == total_balloons).sum()

print(f"Number of cache misses: {cache_miss_count}\n")

df_valid = df[df['data._data'] != "404"]

# Computing thhe difference between 'data._timestamp' e 'msg_receive_timestamp'
df_valid['time_difference_sec'] = (
    (df_valid['msg_receive_timestamp.sec'] - df_valid['data._timestamp.sec']) + 
    (df_valid['msg_receive_timestamp.nanosec'] - df_valid['data._timestamp.nanosec']) / 1e9
)
