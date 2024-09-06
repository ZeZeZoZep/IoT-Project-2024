import pandas as pd
import json


pd.set_option('display.max_rows', None)
# Carica il file JSON
with open("offloaded_data.json") as f:
    data = json.load(f)

# Trasforma i dati in un DataFrame
df_0 = pd.json_normalize(data)

max_polling_sqn = 1000
print("\n***\nVERRANNO ANALIZZATE",max_polling_sqn," RICHIESTE DELLA BASE STATION\n***\n")


df = df_0[(df_0['polling_sqn'] < max_polling_sqn) & (df_0['polling_sqn'] > 0)]

# Visualizza i primi record
#print(df.head())

# Analisi di base
# Numero di record per ogni balloon_id
#balloon_counts = df['balloon_id'].value_counts()
#print(balloon_counts)

# Filtra i record con dati sensoriali validi (non 404)
#valid_sensor_data = df[df['data._data'] != "404"]

# Calcola la media dei tempi di polling per ogni sensore
#df['polling_duration'] = df['msg_receive_timestamp.sec'] + df['msg_receive_timestamp.nanosec'] / 1e9
#average_polling_time = df.groupby('data._sensor_id')['polling_duration'].mean()
#print(average_polling_time)

# Statistiche descrittive dei tempi
#print(df['polling_duration'].describe())

# Esplora la distribuzione dei dati "404"
error_data = df[df['data._data'] == "404"]
print(error_data['balloon_id'].value_counts())

#Counting numbers of cache miss
df_404 = df[df['data._data']=='404']
           
grouped_404 = df_404.groupby('polling_sqn')['balloon_id'].count()

total_balloons = df['balloon_id'].nunique()
cache_miss_count = (grouped_404 == total_balloons).sum()

print(f"Numero di cache miss: {cache_miss_count}")

#df_valid = df
df_valid = df[df['data._data'] != "404"]

# Calcola la differenza temporale tra 'data._timestamp' e 'msg_receive_timestamp' per i messaggi validi
df_valid['time_difference_sec'] = (
    (df_valid['msg_receive_timestamp.sec'] - df_valid['data._timestamp.sec']) + 
    (df_valid['msg_receive_timestamp.nanosec'] - df_valid['data._timestamp.nanosec']) / 1e9
)

#df_valid['time_difference_sec'] = (
#    (df_valid['msg_receive_timestamp.sec'] + (df_valid['msg_receive_timestamp.nanosec']) / 1e9) - 
#    (df_valid['data._timestamp.sec'] + (df_valid['data._timestamp.nanosec']) / 1e9)
#)


if (1):
    print("Differenze temporali calcolate (in secondi):")
    print(df_valid[['data._timestamp.sec', 'msg_receive_timestamp.sec', 
                    'data._timestamp.nanosec', 'msg_receive_timestamp.nanosec', 
                    'time_difference_sec']])
# Calcola il tempo medio per i messaggi validi
average_time_difference_valid = df_valid['time_difference_sec'].mean()

print(f"Tempo medio tra invio richiesta e ricezione per messaggi validi: {average_time_difference_valid:.9f} secondi")
