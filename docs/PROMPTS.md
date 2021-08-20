# Prompts

The commander node features a prompt subsystem which is used by the sequencer to emit prompts when a prompt block is executed. Prompts are used for alerting the user of some information and giving them the option to continue or abort the sequence. Prompts can also allow the user to access 
This uses the following topics and services:

## Published Topics
 - `sequence/prompt` ([commander_msgs/Prompt](../commander_msgs/msg/Prompt.msg))
   - Messages are published to this topic when a prompt is to be show or hidden.
   - Latched so that newly connecting clients will get currently active prompts.

## Advertised Services
 - `sequence/prompt` ([commander_msgs/PromptResponse](../commander_msgs/srv/PromptResponse.srv))
   - Call this service to acknowledge a prompt.

Each prompt is emitted with a UUID, a 128 bit identifier which must be passed in when the prompt response service is called.

If a prompt message is received with a blank UUID field, this means that the current prompt has been dismissed, and the prompt should be removed.
