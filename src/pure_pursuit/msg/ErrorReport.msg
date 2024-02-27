string description # description of the error
int8 severity      # what is the severity of this error [1-5] (see safety_node for details)
string origin      # where this error is coming from (usually just node name)
string module      # did the error come from control? perception? localization?
int8 lifetime      # how long this error should persist (in seconds), -1 is infinite