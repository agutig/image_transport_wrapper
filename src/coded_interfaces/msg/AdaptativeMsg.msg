#Adaptative Message.

string role   #Server - client
uint8 msg_type # 0= handshake/requirements , 2 = status , 3 = sync 

string msg_json 

# Handshake   (type 0) + client content --> Client demands. TO maximice codec efficazy please fill the FEWER demands as they will be considered mandatory
# fps = 30,60 ... nan
# height = ...nan
# weigth = ...nan
# frame_type = progressive/ interlaced nan
# max_bit_rate =   ... nan  bps


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


#sync (type 2) + server --> Refresh how data its being sended.
# sended_at = date

