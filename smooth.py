import pandas as pd
import numpy as np

df = pd.read_csv('NS4218_2.csv')

# --- Fix repeated/stuck values (linear interpolate duplicates) ---
df['spd_ego'] = df['spd_ego'].where(df['spd_ego'] != df['spd_ego'].shift()).interpolate(method='linear')
df['pos_ego'] = df['pos_ego'].where(df['pos_ego'] != df['pos_ego'].shift()).interpolate(method='linear')

# --- Smooth with rolling average (pure pandas, no scipy needed) ---
window = 5  # adjust: larger = smoother

df['spd_ego'] = df['spd_ego'].rolling(window=window, center=True, min_periods=1).mean()
df['pos_ego'] = df['pos_ego'].rolling(window=window, center=True, min_periods=1).mean()

df.to_csv('NS4218_2_clean.csv', index=False)