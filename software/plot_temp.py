import pandas as pd
import matplotlib.pyplot as plt


#KP = 2.0, KI = 0.2, KD = 0.8
input_csv = "./logs/log_20250911_110732.csv"

df = pd.read_csv(
    input_csv,
    usecols=['iso_time', 'temp_c', 'setpoint_c'],
    parse_dates=['iso_time']
)





# poner iso_time como índice de tiempo
df = df.set_index('iso_time')

# downsampling opcional para graficar (ej. cada 100 puntos)
df_plot = df.iloc[::100, :]

df_smooth = df_plot.rolling('10min').mean()

plt.figure(figsize=(10, 6))
plt.plot(df_smooth.index, df_smooth['temp_c'], label='Temperatura', color='blue')
plt.plot(df_smooth.index, df_smooth['setpoint_c'], label='Setpoint', color='red')
plt.xlabel('Tiempo')
plt.ylabel('Temperatura (°C)')
plt.title('Control de temperatura Kp=2.0, Ki=0.1, Kd=0.01')
plt.xticks(rotation=45)
plt.legend()
plt.tight_layout()
plt.grid()
plt.show()
