"""autogenerated by genmsg_py from DiagnosisResults.msg. Do not edit."""
import roslib.message
import struct


class DiagnosisResults(roslib.message.Message):
  _md5sum = "c2339e27ed3da1224754f35abdc3a7fb"
  _type = "diagnosis_msgs/DiagnosisResults"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string[] good
string[] bad

"""
  __slots__ = ['good','bad']
  _slot_types = ['string[]','string[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       good,bad
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(DiagnosisResults, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.good is None:
        self.good = []
      if self.bad is None:
        self.bad = []
    else:
      self.good = []
      self.bad = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      length = len(self.good)
      buff.write(_struct_I.pack(length))
      for val1 in self.good:
        length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.bad)
      buff.write(_struct_I.pack(length))
      for val1 in self.bad:
        length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.good = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1 = str[start:end]
        self.good.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.bad = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1 = str[start:end]
        self.bad.append(val1)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      length = len(self.good)
      buff.write(_struct_I.pack(length))
      for val1 in self.good:
        length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.bad)
      buff.write(_struct_I.pack(length))
      for val1 in self.bad:
        length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.good = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1 = str[start:end]
        self.good.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.bad = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        val1 = str[start:end]
        self.bad.append(val1)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I