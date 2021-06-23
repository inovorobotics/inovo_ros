byte INPUT_NONE=0
byte INPUT_TEXT=1
byte INPUT_NUMBER=2

Header header
string prompt_id # A unique ID representing the prompt - if empty it means the prompt has been cancelled
byte input # The type of input the prompt is expecting
string block_id # The ID of the block showing the prompt
string message # A message for the prompt to display
