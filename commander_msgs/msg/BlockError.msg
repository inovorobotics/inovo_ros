Header header

# The ID of the block which produced this error
# If this is an empty string it means "no error"
# This message is typically used with a latched topic, so this "no error" state is used to notify subsribers that the error is no longer present
string block_id

# A description of the error
string message
