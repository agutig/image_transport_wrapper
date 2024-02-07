# RLEimg.msg

# Original image size
uint32 original_width
uint32 original_height

#coded vector
float32[] rle_values #values 
uint32[] rle_counts  #number of times each value repeats
