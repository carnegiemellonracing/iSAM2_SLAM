# generated from rosidl_generator_py/resource/_idl.py.em
# with input from eufs_msgs:srv/SetMission.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetMission_Request(type):
    """Metaclass of message 'SetMission_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'NOT_SELECTED': 0,
        'ACCELERATION': 1,
        'SKIDPAD': 2,
        'AUTOCROSS': 3,
        'TRACK_DRIVE': 4,
        'MANUAL_DRIVING': 5,
        'ADS_EBS_TEST': 6,
        'ADS_INSPECTION': 7,
        'DDT_INSPECTION_A': 8,
        'DDT_INSPECTION_B': 9,
        'DDT_AUTONOMOUS_DEMO': 10,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('eufs_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'eufs_msgs.srv.SetMission_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_mission__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_mission__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_mission__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_mission__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_mission__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'NOT_SELECTED': cls.__constants['NOT_SELECTED'],
            'ACCELERATION': cls.__constants['ACCELERATION'],
            'SKIDPAD': cls.__constants['SKIDPAD'],
            'AUTOCROSS': cls.__constants['AUTOCROSS'],
            'TRACK_DRIVE': cls.__constants['TRACK_DRIVE'],
            'MANUAL_DRIVING': cls.__constants['MANUAL_DRIVING'],
            'ADS_EBS_TEST': cls.__constants['ADS_EBS_TEST'],
            'ADS_INSPECTION': cls.__constants['ADS_INSPECTION'],
            'DDT_INSPECTION_A': cls.__constants['DDT_INSPECTION_A'],
            'DDT_INSPECTION_B': cls.__constants['DDT_INSPECTION_B'],
            'DDT_AUTONOMOUS_DEMO': cls.__constants['DDT_AUTONOMOUS_DEMO'],
        }

    @property
    def NOT_SELECTED(self):
        """Message constant 'NOT_SELECTED'."""
        return Metaclass_SetMission_Request.__constants['NOT_SELECTED']

    @property
    def ACCELERATION(self):
        """Message constant 'ACCELERATION'."""
        return Metaclass_SetMission_Request.__constants['ACCELERATION']

    @property
    def SKIDPAD(self):
        """Message constant 'SKIDPAD'."""
        return Metaclass_SetMission_Request.__constants['SKIDPAD']

    @property
    def AUTOCROSS(self):
        """Message constant 'AUTOCROSS'."""
        return Metaclass_SetMission_Request.__constants['AUTOCROSS']

    @property
    def TRACK_DRIVE(self):
        """Message constant 'TRACK_DRIVE'."""
        return Metaclass_SetMission_Request.__constants['TRACK_DRIVE']

    @property
    def MANUAL_DRIVING(self):
        """Message constant 'MANUAL_DRIVING'."""
        return Metaclass_SetMission_Request.__constants['MANUAL_DRIVING']

    @property
    def ADS_EBS_TEST(self):
        """Message constant 'ADS_EBS_TEST'."""
        return Metaclass_SetMission_Request.__constants['ADS_EBS_TEST']

    @property
    def ADS_INSPECTION(self):
        """Message constant 'ADS_INSPECTION'."""
        return Metaclass_SetMission_Request.__constants['ADS_INSPECTION']

    @property
    def DDT_INSPECTION_A(self):
        """Message constant 'DDT_INSPECTION_A'."""
        return Metaclass_SetMission_Request.__constants['DDT_INSPECTION_A']

    @property
    def DDT_INSPECTION_B(self):
        """Message constant 'DDT_INSPECTION_B'."""
        return Metaclass_SetMission_Request.__constants['DDT_INSPECTION_B']

    @property
    def DDT_AUTONOMOUS_DEMO(self):
        """Message constant 'DDT_AUTONOMOUS_DEMO'."""
        return Metaclass_SetMission_Request.__constants['DDT_AUTONOMOUS_DEMO']


class SetMission_Request(metaclass=Metaclass_SetMission_Request):
    """
    Message class 'SetMission_Request'.

    Constants:
      NOT_SELECTED
      ACCELERATION
      SKIDPAD
      AUTOCROSS
      TRACK_DRIVE
      MANUAL_DRIVING
      ADS_EBS_TEST
      ADS_INSPECTION
      DDT_INSPECTION_A
      DDT_INSPECTION_B
      DDT_AUTONOMOUS_DEMO
    """

    __slots__ = [
        '_mission',
    ]

    _fields_and_field_types = {
        'mission': 'int16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mission = kwargs.get('mission', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.mission != other.mission:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def mission(self):
        """Message field 'mission'."""
        return self._mission

    @mission.setter
    def mission(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mission' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'mission' field must be an integer in [-32768, 32767]"
        self._mission = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_SetMission_Response(type):
    """Metaclass of message 'SetMission_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('eufs_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'eufs_msgs.srv.SetMission_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_mission__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_mission__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_mission__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_mission__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_mission__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetMission_Response(metaclass=Metaclass_SetMission_Response):
    """Message class 'SetMission_Response'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_SetMission(type):
    """Metaclass of service 'SetMission'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('eufs_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'eufs_msgs.srv.SetMission')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_mission

            from eufs_msgs.srv import _set_mission
            if _set_mission.Metaclass_SetMission_Request._TYPE_SUPPORT is None:
                _set_mission.Metaclass_SetMission_Request.__import_type_support__()
            if _set_mission.Metaclass_SetMission_Response._TYPE_SUPPORT is None:
                _set_mission.Metaclass_SetMission_Response.__import_type_support__()


class SetMission(metaclass=Metaclass_SetMission):
    from eufs_msgs.srv._set_mission import SetMission_Request as Request
    from eufs_msgs.srv._set_mission import SetMission_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
