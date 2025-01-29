""" 

Functionality for reading ROS2 bag files.

ROS2 facilitates data recording and playback via bag_ files. This package 
provides functionality for extracting data stored in bag files, and translating 
it into Python builtin data types.

.. _bag: https://docs.ros.org/en/galactic/Tutorials/Ros2bag
         /Recording-And-Playing-Back-Data.html)

Notes
-----

Although the sqlite3_ package is part of the standard distribution of 
Python 3 -- and it can be used to directly read from bag files -- it is 
recommended that the ROS2 API be used for decoding bag files wherever possible.

.. _sqlite3: https://docs.python.org/3/library/sqlite3.html

"""

# Copyright 2022 Carnegie Mellon University Neuromechatronics Lab
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

# Contact: a.whit (nml@whit.contact)


# Import collection interfaces.
from collections.abc import Iterator
from collections.abc import Sequence

# Import ROS2 functionality for reading from bag files.
from rosbag2_py import StorageOptions
from rosbag2_py import ConverterOptions
from rosbag2_py import SequentialReader
from rosbag2_py import StorageFilter

# Import ROS2 functionality for deserializing and translating messages.
#from . import message as message_manipulation
from nml_bag import message as message_manipulation


class Reader(SequentialReader):
    """ A ROS2 bag sequential reader class with an iterator interface that 
        automatically converts messages to standard Python builtin data types.
    
    Subclass of SequentialReader_, from the ros2bag_ package.
    
    .. _SequentialReader: https://github.com/ros2/rosbag2/blob/master
                          /rosbag2_cpp/src/rosbag2_cpp/readers
                          /sequential_reader.cpp
    .. _ros2bag: https://github.com/ros2/rosbag2#rosbag2
    
    Parameters
    ----------
    *args : list
        Arguments passed to superclass constructor_.
        
        .. _constructor: https://github.com/ros2/rosbag2/blob
                         /fada54cc2c93d4238f3971e9ce6ea0006ac85c8c/rosbag2_cpp
                         /src/rosbag2_cpp/readers/sequential_reader.cpp#L60
    filepath : str
        Bag file path to open. If not specified, then the :py:func`open` 
        function must be invoked before accessing any other class functionality.
    topics : list of strings
        A list of topics to read. Other topics will be excluded.
    *kargs : dict
        Keyword arguments passed to superclass constructor_.
        
        .. _constructor: https://github.com/ros2/rosbag2/blob
                         /fada54cc2c93d4238f3971e9ce6ea0006ac85c8c/rosbag2_cpp
                         /src/rosbag2_cpp/readers/sequential_reader.cpp#L60
                  
    """  # noqa: D205, D210, D213, D412, W293
    
    def __init__(self, filepath=None, topics=[], storage_id='mcap', 
                 serialization_format='cdr', get_msg_count = False ,**kwargs):
        super().__init__(**kwargs) # SequentialReader
        if filepath: self.open(filepath, storage_id=storage_id, 
                               serialization_format=serialization_format)
        if topics: self.set_filter(topics)
        
        # Keep instance wise records
        self.topics_list = topics
        self._msg_count_dict = {}

        # print message counts
        if get_msg_count:
            self.get_msg_count()

        
    def open(self, filepath, storage_id='mcap', serialization_format='cdr'):
        """ Open a bag file.
        
        Parameters
        ----------
        filepath : str
            Filesystem path to the bag file to open.
        storage_id: str
            The storage plugin to use. See the ROS2 bag documentation for more information.
        serialization_format: str
            The serialization format to use. See the ROS2 bag documentation for more information.
        
        Notes
        -----
        
        By default_, the ROS2 bag mechanism uses the sqlite_ 3 storage format, 
        and the Common Data Representation (CDR_) serialization format.
        
        .. _default: https://github.com/ros2
                     /rosbag2#storage-format-plugin-architecture
        .. _sqlite: https://en.wikipedia.org/wiki/SQLite
        .. _CDR: https://en.wikipedia.org/wiki/Common_Data_Representation

        """
        storage_options = StorageOptions(uri=filepath, storage_id=storage_id)
        converter_options = ConverterOptions(input_serialization_format=serialization_format,
                                             output_serialization_format=serialization_format)
        super().open(storage_options, converter_options)
        #self._records = list(self)
        
    @property
    def topics(self):
        """ A list of ROS topics_ recorded in the open bag file.
        
        .. _topics: https://docs.ros.org/en/galactic/Tutorials/Topics
                    /Understanding-ROS2-Topics.html#background
        """
        return list(self.type_map)
    
    @property
    def type_map(self):
        """ A dict that maps ROS2 topic names to message types, for all 
            topics recorded in the open bag file.
        
        Message types are specified as strings.
        
        See ROS2 background_ documentation for further explanation of nodes and 
        messages.
        
        .. _background: https://docs.ros.org/en/humble/Tutorials/Topics
                        /Understanding-ROS2-Topics.html#background
        """
        topic_data = self.get_all_topics_and_types()
        return {metadata.name: metadata.type for metadata in topic_data}
        
    @property
    def records(self):
        """ A list of all messages (after filtering), in records format.
        
        Each element of the list is a dict mapping between a message field 
        name and the value provided by the message.
        
        Notes
        -----
        Practice caution, as accessing this property will read all available 
        messages into an expanded records format. This is **not** a 
        memory-efficient format. For large bag files, the memory burden can 
        cause issues. This is especially true if no message filters are set.
        """
        if not getattr(self, '_records', None): self._records = list(self)
        return self._records
    
    def set_filter(self, topics):
        """ Set the topic filter.
        
        Parameters
        ----------
        topics : list of strings
            A list of the topics to include.
        """
        super().set_filter(StorageFilter(topics))
    
    @property
    def msg_count_dict(self):
        """
        Getter for msg_count_dict.
        """
        return self._msg_count_dict

    def get_msg_count(self):
        """For the given topics, count and return number of messages as a dictionary"""
        
        self._msg_count_dict = {}  # Reset the dictionary
        # Iterate through all messages
        while self.has_next():
            (topic, _, _) = self.read_next()
            if topic in self.topics_list:
                self._msg_count_dict[topic] = self._msg_count_dict.get(topic, 0) + 1
        return self._msg_count_dict

    # __iter__ should be implemented by collections.abc.Iterator.
    def __iter__(self): return self
    
    def __next__(self):
        """ Return the next_ message, in record format.
        
        .. _next: https://docs.python.org/3/library
                   /stdtypes.html#iterator.__next__
        
        Returns
        -------
        record : dict
            A dict mapping with keys that match the fields 
            specified in the message definition, along with the following:
            
            * `topic`: The ROS topic (string) the message was received on.
            * `time_ns`: The message timestamp in nanoseconds.
            * `type`: The ROS message type.
        """
        if not self.has_next(): raise StopIteration()
        
        # Read the next message.
        (topic, data, time) = self.read_next()
        
        # Convert the serialized message data to an OrderedDict.
        message_type = self.type_map[topic]
        data = message_manipulation.to_dict(data, message_type)
        
        # For simplicity, the reader assumes that the message does not contain 
        # fields that conflict with the message metadata.
        assert('topic' not in data)
        assert('time_ns' not in data)
        assert('type' not in data)
        
        # Convert to record format.
        record = {'topic': topic, 'time_ns': time, 'type': message_type, **data}
        
        # Return the result.
        return record
    
  

if __name__ == '__main__':
    import doctest
    option_flags = doctest.NORMALIZE_WHITESPACE | doctest.ELLIPSIS
    doctest.testmod(optionflags=option_flags)
    
  

