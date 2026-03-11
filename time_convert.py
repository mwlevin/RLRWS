def moy_to_hour_min(moy):
    # convert minute of the year to hour and minute of the year
    hour = int(moy/60)
    minute = moy - hour*60
    return hour, minute


def ms_to_sec(ms):
    # convert milisecond of the minute to second of the minute
    sec = ms/1000
    return sec

def tenth_s_to_min_sec(tenth_s):
    # convert xx 0.1s of the hour to minute and second of the hour
    minute = int((tenth_s/10)/60)
    sec = tenth_s/10 - 60*minute
    return minute, sec

def calc_change_time(minute_chagne, sec_change, moy_minute, sec_of_minute):
    sec_to_chagne = (minute_chagne - moy_minute)*60+sec_change - sec_of_minute
    return sec_to_chagne

# def moy_to_date(moy):
#     hour, minute = moy_to_hour_min(moy)
#     if hour <= 31*24:
#         month = 1
#         date = 
#     elif hour<= 31*24 + 28*24:
#         month = 2
#     elif hour<= 31*24 + 28*24 + 31*24:
#         month = 3
#     elif hour<= 31*24 + 28*24 + 31*24 + 30*24:
#         month = 4
#     elif hour<= 31*24 + 28*24 + 31*24 + 30*24 + 31*24:
#         month = 5
#     elif hour<= 31*24 + 28*24 + 31*24 + 30*24 + 31*24 + 30*24:
#         month = 6
#     elif hour<= 31*24 + 28*24 + 31*24 + 30*24 + 31*24 + 30*24 + 31*24:
#         month = 7
#     elif hour<= 31*24 + 28*24 + 31*24 + 30*24 + 31*24 + 30*24 + 31*24 + 31*24:
#         month = 8
#     elif hour<= 31*24 + 28*24 + 31*24 + 30*24 + 31*24 + 30*24 + 31*24 + 31*24 + 30*24:
#         month = 9
#     elif hour<= 31*24 + 28*24 + 31*24 + 30*24 + 31*24 + 30*24 + 31*24 + 31*24 + 30*24 + 31*24:
#         month = 10
#     elif hour<= 31*24 + 28*24 + 31*24 + 30*24 + 31*24 + 30*24 + 31*24 + 31*24 + 30*24 + 31*24 + 30*24:
#         month = 11
#     elif hour<= 31*24 + 28*24 + 31*24 + 30*24 + 31*24 + 30*24 + 31*24 + 31*24 + 30*24 + 31*24+ 30*24 + 31*24:
#         month = 12
#     else:
#         month = None
    
    