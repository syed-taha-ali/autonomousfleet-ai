# Status feedback emitted by mission_manager_node
string mission_id
string state            # idle | planning | executing | paused | completed | failed
string current_action
float32 progress        # 0.0 - 1.0
string error_message
builtin_interfaces/Time stamp
