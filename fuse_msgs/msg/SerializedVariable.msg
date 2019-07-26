std_msgs/Header header  # A header, because it is always nice to have a timestamp
string plugin_name      # The fully-qualified name of the plugin used to serialize/deserialize this variable
string data             # The serialized variable as a string. This is for demonstration purposes only. In reality
                        # we would want to serialize using a binary format, so this should be uint8[] or similar.
