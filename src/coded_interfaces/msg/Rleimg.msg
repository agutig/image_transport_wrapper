# Rleimg.msg

# Original image size
uint16 original_width
uint16 original_height

#coded vector
int32[] rle_values #values 
uint16[] rle_counts  #number of times each value repeats
