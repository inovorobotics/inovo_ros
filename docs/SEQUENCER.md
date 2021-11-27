# Sequencer

The sequencer subsystem allows users to program the robot using Blockly. Blockly programs are comprised of blocks (naturally) which can and can do any number of things such as controlling the robot, controlling grippers and other devices, and manipulating the state of the running program. For more information on Blockly, visit the [blockly website](https://developers.google.com/blockly).

Blockly programs are transferred to the sequencer in Blockly's XML format using the `/sequence/upload` service, where it is parsed and stored in a local buffer ready to be run (this is called the "working sequence"). The working sequence can then be controlled using the `sequence/start`, `/sequence/pause`, `/sequence/stop` family of services (see full descriptions of these services below).

The working sequence can be modified through the `upload` service as well as the `open_project`, `now_project` family of services. When the working sequence is modified, a copy is published on the latched topic `/sequence/blockly`, so observing clients can update their own copies. There is a problem that subscribed client will get a copy regardless of whether they were the one that just modified it, so client can put a token in the upload message which is passed back on the blockly topic so clients can filter out their own modifications.

## Published Topics
- `/sequence/blockly ` ([commander_msgs/Blockly](../commander_msgs/msg/Blockly.msg))
  - Shows the current state of the working sequence.
  - The Blockly tree is pushed in the Blockly XML format to this topic when the sequence changes.
  - This message contains a Blockly XML string and a token which is populated with the token that was passed in with the `sequence/upload` service, if the Blockly was most recently updated this way.
  - If the Blockly was updated due to some other method such as loading a saved sequence, loading a new sequence, or the first time the sequence is loaded from an autosave, the token will be an empty string, indicating that all UIs will need to update their state of the tree.
  - This topic is latched so late subscribers will get the latest state of the Blockly tree.
  - Also contains information about whether the working sequence has been saved to disk or contains unsaved modifications & the name of the assicated project on disk.

- `/sequence/errors ` ([commander_msgs/BlockError](../commander_msgs/msg/BlockError.msg))
  - If a block emits an error, a message will be published to this topic showing the contents of the error, and the ID of the block that generated it.
  - This topic is latched, so late subscribers will get active errors.
  - When the error is cleared, the a new message is published with an empty block id string. If you receive a message with an empty block ID, this indicates that the error has now disappeared, and should be removed from any display.

- `/sequence/log ` ([commander_msgs/BlockLog](../commander_msgs/msg/BlockLog.msg))
  - If a block prints something, a message will be published to this topic showing the contents of the message, and the ID of the block that printed it.

## Services
- `sequence/upload` ([commander_msgs/Upload](../commander_msgs/srv/Upload.srv))
   - Call this service to upload a sequence in the Blockly XML format.
   - The payload of the message takes a string which contains the entire new state of the Blockly XML, and a token which is a unique ID for the client sending the Blockly.
   - The token is used so that UIs can ignore their own messages when the Blockly sequence is received again from the `sequence/blockly` topic.

- `/sequence/open_project` ([commander_msgs/Project](../commander_msgs/srv/Project.srv))
   - Loads a project from disk into the working sequence.
   - Call the `/sequence/list_projects` service to get a list of projects which can be loaded.

- `/sequence/save_project` ([commander_msgs/Project](../commander_msgs/srv/Project.srv))
   - Saves the sequence to disk.
   - If the filename field is left empty, the currently loaded project will be overwritten. The filename of this project can be found in the `/sequence/blockly` topic.
   - If the filename field is populated, the working sequnce will be saved to a new project.

- `/sequence/list_projects` ([commander_msgs/ListProjects](../commander_msgs/srv/ListProjects.srv))
   - Shows a list of projects which are currently saved to disk - any of these projects can be passed to `/sequence/open_project`.

- `/sequence/new_project` ([std_srvs/Trigger](http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html))
   - Clears the working sequence and removes project association.
