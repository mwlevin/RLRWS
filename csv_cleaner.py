import re
import pandas as pd
from io import StringIO

input_path  = 'calibrate_data.csv'   # change path if needed
output_path = 'calibrate_data_clean.csv'

with open(input_path, 'r') as f:
    raw = f.read()

# Collapse newlines that are inside quoted fields
# (real row boundaries start with a timestamp or the header word)
cleaned = re.sub(r'\n(?!\d{2}:\d{2}|\d{4}-\d{2}-\d{2}|timestamp)', ' ', raw)

df = pd.read_csv(StringIO(cleaned))
print(f"Columns ({len(df.columns)}): {df.columns.tolist()}")
print(f"Rows: {len(df)}")

# Extract just the first number from predicted_tl_state (e.g. [[1.]\n[1.]] -> 1.0)
df['predicted_tl_state'] = df['predicted_tl_state'].apply(
    lambda x: float(re.findall(r'[\d.]+', str(x))[0]) if re.findall(r'[\d.]+', str(x)) else 1.0
)

df.to_csv(output_path, index=False)
print(f"Saved clean file to: {output_path}")