# banana_interfaces

Custom message definitions for BananaHand.

Structure
- `msg/` for `.msg` files

Current messages
- `HandState.msg`

Notes
- Add new messages to `CMakeLists.txt` in the `msg_files` list.
- When you add messages, run `colcon build` to generate interfaces.
