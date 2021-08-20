# Sequencer

The sequencer subsystem allows users to program the robot using a tree of blocks to describe the program. A block is a single atomic operation and can do any number of things such as controlling the robot, controlling grippers and other devices, and manipulating the state of the running sequence.

Sequences are described to the sequencer using Blockly trees. These are transferred to the sequencer using the Blockly XML format using the `sequence/Upload` service.

## Published Topics
- `/sequence/errors ` ([commander_msgs/BlockError](../commander_msgs/msg/BlockError.msg))
  - If a block emits an error, a message will be published to this topic showing the contents of the error, and the ID of the block that generated it.
  - This topic is latched, so late subscribers will get active errors.
  - When the error is cleared, the a new message is published with an empty block id string. If you receive a message with an empty block ID, this indicates that the error has now disappeared, and should be removed from any display.

- `/sequence/log ` ([commander_msgs/BlockLog](../commander_msgs/msg/BlockLog.msg))
  - If a block prints something, a message will be published to this topic showing the contents of the message, and the ID of the block that printed it.

- `/sequence/blockly ` ([commander_msgs/Blockly](../commander_msgs/msg/Blockly.msg))
  - The Blockly tree is pushed in the Blockly XML format to this topic when the sequence changes.
  - This message contains a Blockly XML string and a token which is populated with the token that was passed in with the `sequence/upload` service, if the Blockly was updated this way.
  - If the Blockly was updated due to some other method such as loading a saved sequence, loading a new sequence, or the first time the sequence is loaded from an autosave, the token will be an empty string, indicating that all UIs will need to update their state of the tree.
  - This topic is latched so late subscribers will get the latest state of the Blockly tree.

## Advertised Services
- `sequence/upload` ([commander_msgs/Upload](../commander_msgs/srv/Upload.srv))
   - Call this service to upload a sequence in the Blockly XML format.
   - The payload of the message takes a string which contains the entire new state of the Blockly XML, and a token which is a unique ID for the client sending the Blockly.
   - The token is used so that UIs can ignore their own messages when the Blockly sequence is received again from the `sequence/blockly` topic.
