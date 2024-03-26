# Adaptative.msg

string role   # Server or Client
uint8 msg_type  # 0=Handshake/Requirements, 1=Status, 2=Sync (parece haber un error en la secuencia original, asumo que querías incluir el número 1)

string msg_json  # JSON formatted message

# Handshake   (type 0) + client content --> Client demands. TO maximice codec efficazy please fill the FEWER demands as they will be considered mandatory
# fps = 30,60 ... nan
# height = ...nan
# weigth = ...nan
# frame_type = progressive/ interlaced nan
# max_bit_rate =   ... nan  bps
# procesing_time =...0.1 byd efault


# Handshake   (type 0) + server content --> Acept/reject request
# acepted = true/false
# error_msg = "the connection were unable due to..."


#status (type 1) + client --> Refresh how data its being sended.
# requested_bitrate


#status (type 1) + server --> Refresh how data its being sended.
# fps = 30,60 ...
# height =
# weigth =
# frame_type = progressive/ interlaced
# max_bit_rate =    bps
# procesing_time =...


#sync (type 2) + server --> Refresh how data its being sended.
# sended_at = date

