import re
from collections import namedtuple
from io import StringIO

class dicti(dict):
    """Dictionary that enables case insensitive searching while preserving case sensitivity 
    when keys are listed, i.e., via keys() or items() methods. 
    
    Works by storing a lowercase version of the key as the new key and stores the original key-value 
    pair as the key's value (values become dictionaries).
    Adjusted from https://gist.github.com/babakness/3901174"""

    _kv = namedtuple('kv','key val')

    def __init__(self, initval={}):
        if isinstance(initval, dict):
            for key, value in initval.iteritems():
                self.__setitem__(key, value)
        elif isinstance(initval, list):
            for (key, value) in initval:
                self.__setitem__(key, value)
            
    def __contains__(self, key):
        return dict.__contains__(self, key.lower())
  
    def __getitem__(self, key):
        return dict.__getitem__(self, key.lower()).val
  
    def __setitem__(self, key, value):
        return dict.__setitem__(self, key.lower(), self._kv(key, value))

    def get(self, key, default=None):
        try:
            v = dict.__getitem__(self, key.lower())
        except KeyError:
            return default
        else:
            return v.val

    def has_key(self,key):
        if self.get(key):
            return True
        else:
            return False    

    def items(self):
        return [(v.key, v.val) for v in dict.itervalues(self)]
    
    def keys(self):
        return [v.key for v in dict.itervalues(self)]
    
    def values(self):
        return [v.val for v in dict.itervalues(self)]
    
    def iteritems(self):
        for v in dict.itervalues(self):
            yield v.key, v.val
        
    def iterkeys(self):
        for v in dict.itervalues(self):
            yield v.key
        
    def itervalues(self):
        for v in dict.itervalues(self):
            yield v.val
    
    def copy(self):
        return dict(self.iteritems())

class ParsingError(Exception):
    pass

class MissingDefinitionError(Exception):
    pass

class DefFile(object):
    def __init__(self,format=None, manifest=True):
        self.format = format
        if manifest is True:
            self.manifest = Manifest()
        elif manifest:
            self.manifest = manifest
        else:
            self.manifest = None
        self.definitions = []
        self.sections = dicti()
    
    @property
    def type(self):
        if not self.manifest:
            return None
        return self.manifest.def_type
    
    @type.setter
    def type(self, value):
        if not self.manifest:
            self.manifest = Manifest()
        self.manifest.def_type = value
    
    def definition_types(self):
        types = set()
        for definition in self.definitions:
            types.add(definition.type)
        return list(types)
    
    def get_definition(self,dfn_type, name):
        for d in self.definitions:
            if d.type == dfn_type and d.name == name:
                return d
        return None
    
    def get_definitions_of_type(self,dfn_type):
        return [d for d in self.definitions if d.type == dfn_type]
    
    def add_definition(self,dfn,quiet=False):
        for d in self.definitions:
            if d.type == dfn.type and d.name == dfn.name:
                if quiet:
                    return
                else:
                    raise TypeError("%s:%s definition already exists" % (dfn.type, dfn.name))
        self.definitions.append(dfn)
    
    def has_section(self,section, format=None):
        return self.sections.has_key(section)
    
    def get_section(self,section, format=None):
        sec = self.sections.get(section)
        if sec and format and sec.format != format:
            return None
        else:
            return sec
    
    def add_section(self,section):
        self.sections[section.name] = section
    
    def load_includes(self,path=None):
        pass
    
    def tojson(self):
        d = {}
        
        d['Manifest'] = self.manifest.tojson()
        
        d['Definitions'] = dict([(dfn.type + ':' + dfn.name, dfn.tojson()) for dfn in self.definitions])
        
        d['Sections'] = dict([(section_name, section.tojson()) for section_name, section in self.sections.iteritems()])
        
        return d
    
    def tostring(self,suppress_formats=False):
        strs = []
        if self.manifest:
            s = ''
            if self.format and not suppress_formats:
                s += '[Manifest!%s]\n' % self.format
            else:
                s += '[Manifest]\n'
            s += self.manifest.__str__()
            strs.append(s)
        elif self.format and not suppress_formats:
            strs.append('[Manifest!%s]' % self.format)
        
        for dfn in self.definitions:
            s = ''
            if dfn.format and not suppress_formats:
                s += '[%s:%s!%s]\n' % (dfn.type, dfn.name, dfn.format)
            else:
                s += '[%s:%s]\n' % (dfn.type, dfn.name)
            s += dfn.__str__()
            strs.append(s)
        
        for sec_name, section in self.sections.iteritems():
            s = ''
            sec_name = section.name
            if sec_name.find(':') != -1:
                sec_name = ':' + sec_name
            if section.format and not suppress_formats:
                s += '[%s!%s]\n' % (sec_name, section.format)
            else:
                s += '[%s]\n' % sec_name
            s += section.__str__()
            strs.append(s)
        
        return '\n\n'.join(strs)
    
    def get(self, key, default=None):
        if isinstance(key, tuple):
            sec = self.get_section(key[0])
            if not sec: return default
            if len(key) == 2:
                return sec[key[1]]
            else:
                return sec.__getitem__(key[1:])
        else:
            if not self.manifest:
                return default
            else:
                return self.manifest[key]
    
    def __getitem__(self, key):
        return self.get(key)
    
    def __str__(self):
        return self.tostring()
                

class Manifest(object):
    def __init__(self):
        self.def_type = None
        self.includes = []
        self.fields = dicti()
    
    def __getitem__(self,key):
        return self.fields.get(key)
    
    def __setitem__(self,key,value):
        self.fields[key] = value
        
    def tojson(self):
        d = dict(self.fields.iteritems())
        d['Def-Type'] = self.def_type
        return d
    
    def tostring(self):
        strs = []
        if self.def_type is not None:
            strs.append('Def-Type = %s' % self.def_type)
        for include_file, include_type in self.includes:
            if include_type:
                strs.append('Include.%s = %s' % (include_type, include_file))
            else:
                strs.append('Include = %s' % include_file)
        for key, value in self.fields.iteritems():
            strs.append('%s = %s' % (key,value))
        return '\n'.join(strs)
    
    def __str__(self):
        return self.tostring()

class Definition(object):
    Field = namedtuple('Field', ['name','type'])
    FORMAT = None
    
    def __init__(self,type,name):
        self.type = type
        self.name = name
    
    @property
    def format(self):
        if not hasattr(self, '_format'):
            return self.FORMAT
        else:
            return self._format
    @format.setter
    def format(self,value):
        setattr(self,'_format',value)
    
    def tojson(self):
        raise NotImplementedError()
    
    def tostring(self):
        raise NotImplementedError()
    
    def __str__(self):
        return self.tostring()

class Section(object):
    FORMAT = None
    def __init__(self,name):
        self.name = name
    
    @property
    def format(self):
        if not hasattr(self, '_format'):
            return self.FORMAT
        else:
            return self._format
    @format.setter
    def format(self,value):
        setattr(self,'_format',value)
    
    def tojson(self):
        raise NotImplementedError()
    
    def tostring(self):
        raise NotImplementedError()
    
    def __str__(self):
        return self.tostring()

class ROSStyleDefinition(Definition):
    FORMAT = 'ros'
    def __init__(self,type,name,segment_names):
        super(ROSStyleDefinition,self).__init__(type,name)
        
        self.segments = tuple([[] for _ in xrange(len(segment_names))])
        self.segment_names = segment_names
    
    def segment(self,index):
        if not isinstance(index,basestring):
            return self.segments[index]
        for idx, seg_name in enumerate(self.segment_names):
            if seg_name == index:
                return self.segment(idx)
        raise IndexError("No segment named %s" % index)
             
    def __getitem__(self,index):
        if len(self.segments) == 1:
            return self.segments[0][index]
        else:
            return self.segment(index)
    
    def tojson(self):
        segs = []
        for segment in self.segments:
            segs.append(list(segment))
        if len(self.segments) == 1:
            return segs[0]
        else:
            return dict(zip(self.segment_names,segs))
    
    def tostring(self):
        segment_strs = []
        for segment in self.segments:
            strs = []
            for name, type in segment:
                strs.append('%s %s' % (type, name))
            segment_strs.append('\n'.join(strs))
        return '\n----\n'.join(segment_strs)

class INISection(Section):
    FORMAT = 'ini'
    def __init__(self,name):
        super(INISection,self).__init__(name)
        self.fields = {}
    
    def has_key(self, key):
        return self.fields.has_key(key)
    
    def get(self, key, default=None):
        return self.fields.get(key, default)
    
    def __getitem__(self, key):
        """If the section has the key, return a (possibly empty) string.
        Otherwise, return None."""
        if self.fields.has_key(key):
            return self.fields.get(key) or ''
        else:
            return None
    
    def __setitem__(self, key, value):
        self.fields[key] = value
    
    def __iter__(self):
        return self.fields.iteritems()
    
    def tojson(self):
        return self.fields.copy()
    
    def tostring(self):
        strs = []
        for key, value in self.fields.iteritems():
            if value.find('\n') != -1:
                strs.append("%s = |" % key)
                [strs.append(line) for line in value.split('\n')]
            else:
                strs.append('%s = %s' % (key,value))
        return '\n'.join(strs)
    
    @staticmethod
    def load_ini_dict(section_dict):
        ini = {}
        for section_name, section in section_dict.iteritems():
            if not isinstance(section, INISection): continue
            for key, value in section.fields:
                ini[section_name + '.' + key] = value
        return ini

class RawSection(Section):
    FORMAT = 'raw'
    def __init__(self,name):
        super(INISection,self).__init__(name)
        self.content = ''
    
    def tojson(self):
        self.content
    
    def tostring(self):
        return self.content


class DefFileParser(object):
    DEF_TYPE_PATT = r'(?P<dfn_type>[a-zA-Z_]\w*)'
    SEC_NAME_PATT = r'(?P<section>(?:[^][\\!#]|(?:\\[][\\!#]))+)'
    FORMAT_PATT = r'(?P<format>[a-zA-Z_]\w*)'
    COMMENT_PATT = r'(?:#(?P<comment>.*))?'
    
    SECTION_PATT = r'\[\s*(?:(?:{dfn_type})?\s*:\s*)?{section}(?:\s*!\s*{format})?\s*\]'.format(
            dfn_type=DEF_TYPE_PATT,section=SEC_NAME_PATT,format=FORMAT_PATT)
    SECTION_RE = re.compile(SECTION_PATT)
    
    KEY_VALUE_PATT_TEMPLATE = r'^(?P<key>{key_patt})\s*=\s*(?P<value>{value_patt})$'
    
    BASIC_KEY_PATT = r'[^\s=](?:\s*\S)*'
    BASIC_VALUE_PATT = r'(?:\S(?:\s*\S)*)?'
    BASIC_KEY_VALUE_PATT = KEY_VALUE_PATT_TEMPLATE.format(key_patt=BASIC_KEY_PATT,value_patt=BASIC_VALUE_PATT)
    BASIC_KEY_VALUE_RE = re.compile(BASIC_KEY_VALUE_PATT)
    
    @classmethod
    def get_section_data(cls,line):
        match = cls.SECTION_RE.match(line)
        if not match:
            return None
        else:
            data = match.groupdict(None)
            data['section'] = re.sub(r'\\(?!\\)','',data['section'])
        return data
    
    @classmethod
    def is_section_line(cls,line):
        return bool(cls.SECTION_RE.match(line))
    
    def __init__(self,default_format=None, default_def_type=None, require_def_type=None):
        self.default_format = default_format
        self.default_def_type = default_def_type
        self.require_def_type = require_def_type or default_def_type
        self.null_section_parser = None
        
        self.definition_parsers = {}
        self.section_parsers = {}
        
        self.fp = None
        self.current_line = 0
        self.null_section = True
    
    def add_section_parser(self,parser, name, format, def_file_format=None):
        self.section_parsers[(name,format,def_file_format)] = parser
    
    def add_default_section_parser(self,parser = None, def_file_format=None, null_section=False):
        parser = parser or RawSectionParser
        self.section_parsers[(None,None,def_file_format)] = parser
        if null_section:
            self.set_null_section_parser(parser)
    
    def set_null_section_parser(self,parser):
        self.null_section_parser = parser
    
    def get_section_parser(self,name,format,def_file_format):
        matches = {}
        for key, parser in self.section_parsers.iteritems():
            parser_name, parser_format, parser_def_file_format = key
            if (parser_name and parser_name != name) or \
                    (parser_format and parser_format != format) or \
                    (parser_def_file_format and parser_def_file_format != def_file_format):
                continue
            score = 0
            if parser_format == format:
                score +=1
            if parser_name:
                score += 2
            matches[score] = parser
        if not matches:
            if format:
                raise ParsingError('No section parser for section %s and format %s' % (name,format))
            else:
                raise ParsingError('No section parser for section %s' % name)
        return matches[max(matches.keys())](name,format,self._get_reader())
    
    def add_definition_parser(self,parser, dfn_type, format, def_file_format=None):
        if not dfn_type:
            raise TypeError('dfn_type cannot be None!')
        self.definition_parsers[(dfn_type,format,def_file_format)] = parser
    
    def get_definition_parser(self,dfn_type,name,format,def_file_format):
        matches = {}
        for key, parser in self.definition_parsers.iteritems():
            parser_def_type, parser_format, parser_def_file_format = key
            if (parser_def_type != dfn_type) or \
                    (parser_format and parser_format != format) or \
                    (parser_def_file_format and parser_def_file_format != def_file_format):
                continue
            score = 0
            if parser_format == format:
                score +=1
            matches[score] = parser
        if not matches:
            if format:
                raise ParsingError('No definition parser for type %s and format %s' % (dfn_type,format))
            else:
                raise ParsingError('No definition parser for type %s' % dfn_type)
        return matches[max(matches.keys())](dfn_type,name,format,self._get_reader())
    
    def readline(self,new=True):
        if self.current_line == 0 or new:
            line = self.fp.readline()
            if not line:
                self.current_line = None
            else:
                if line.endswith('\n'):
                    line = line[:-1]
                self.current_line = re.sub(self.COMMENT_PATT,'',line).strip()
        if self.current_line == '':
            return self.readline()
        return self.current_line
    
    def sections(self):
        line = self.readline(new=False)
        while line is not None:
            section_info = self.get_section_data(line)
            if not section_info:
                if self.null_section and self.null_section_parser:
                    parser = self.null_section_parser(None,None,self._get_reader())
                    sec = parser.parse()
                    section_info = {'section': None, 'data': sec}
                else:
                    raise ParsingError('Parsing error: invalid section line\n%s' % line)
            self.null_section = False
            yield section_info
            line = self.readline(new=False)
    
    def _get_reader(self):
        def reader():
            line = self.readline()
            while line and not DefFileParser.is_section_line(line):
                yield line
                line = self.readline()
        return reader
    
    def parse(self,fp,**kwargs):
        if isinstance(fp,basestring):
            if fp.find('\n') == -1:
                fp = open(fp)
            else:
                fp = StringIO(unicode(fp))
        self.fp = fp
        self.current_line = 0
        self.null_section = True
        default_format = kwargs.get('default_format',self.default_format)
        
        def_file = DefFile(manifest=False)
        for section in self.sections():
            if section['section'] is None:
                def_file.sections[''] = section['data']
            elif section['section'].lower() == 'manifest':
                if def_file.definitions or def_file.sections:
                    raise ParsingError('Parsing error: manifest section must be first!')
                def_file_format = None
                if section['format']:
                    def_file_format = section['format']
                else:
                    def_file_format = default_format
                def_file.format = def_file_format
                parser = DefFileParser.ManifestParser(section['section'],None,self._get_reader())
                def_file.manifest = parser.parse()
            elif section['dfn_type']:
                parser = self.get_definition_parser(section['dfn_type'],section['section'], section['format'],def_file.format)
                definition = parser.parse()
                def_file.definitions.append(definition)
            else:
                parser = self.get_section_parser(section['section'], section['format'],def_file.format)
                def_file.sections[section['section']] = parser.parse()
        if not def_file.type and self.default_def_type:
            def_file.type = self.default_def_type
        if self.require_def_type and not re.match(self.require_def_type,def_file.type):
            raise ParsingError("Require def file type %s, got %s" % (self.require_def_type,def_file.type))
        self.fp = None
        self.current_line = 0
        return def_file
                
    class Subparser(object):
        def __init__(self,name,format,reader):
            self.name = name
            self.format = format
            self.reader = reader
        
        def parse(self):
            raise NotImplementedError()
    
    class SectionParser(Subparser):
        pass
    
    class DefinitionParser(Subparser):
        def __init__(self,dfn_type,name,format,reader):
            super(DefFileParser.DefinitionParser,self).__init__(name,format,reader)
            self.dfn_type = dfn_type
    
    class ManifestParser(Subparser):
        def parse(self):
            mf = Manifest()
            for line in self.reader():
                match = DefFileParser.BASIC_KEY_VALUE_RE.match(line)
                if not match:
                    raise ParsingError('Parsing error: invalid manifest line\n{line}'.format(line=line))
                key = match.group('key')
                value = match.group('value')
                if key.lower() == 'def-type':
                    mf.type = value
                elif key.startswith('include'):
                    dot_loc = key.find('.')
                    if dot_loc == -1:
                        mf.includes.append((value,None))
                    else:
                        include_type = key.split('.')[dot_loc+1:]
                        mf.includes.append((value,include_type))
                else:
                    mf.fields[key] = value
            return mf
        

class ROSStyleDefinitionParser(DefFileParser.DefinitionParser):
    TYPE_PATT = r'\S+'
    NAME_PATT = r'\S+'
    ALLOW_EMPTY_SEGMENTS = True
    
    @classmethod
    def line_patt(cls):
        return r'^(?P<type>{type_patt})\s+(?P<name>{name_patt})$'.format(type_patt=cls.TYPE_PATT,name_patt=cls.NAME_PATT)
    
    SEGMENT_NAMES = None
    
    def parse(self):
        line_re = re.compile(self.line_patt())
        
        rosdef = ROSStyleDefinition(self.dfn_type,self.name,self.SEGMENT_NAMES)
        
        num_segments = len(self.SEGMENT_NAMES)
        segs = []
        
        segment = []
        segment_num = 0
        seg_line_change = True
        for line in self.reader():
            if line == '----':
                segs.append(segment)
                segment = []
                segment_num += 1
                seg_line_change = True
                continue
            seg_line_change = False
            match = line_re.match(line)
            if not match:
                raise ParsingError('Parsing error: invalid ROS definition line\n{line}'.format(line=line))
            segment.append(Definition.Field(name=match.group('name'),type=match.group('type')))
        if seg_line_change:
            if not self.ALLOW_EMPTY_SEGMENTS:
                raise ParsingError('Parsing error: empty segment')
        else:
            segs.append(segment)
        if num_segments and len(segs) != num_segments:
            raise ParsingError('Parsing error: ROS definition needs %d segments' % num_segments)
        rosdef.segments = tuple(segs)
        return rosdef

def get_ros_style_msg_parser(type_patt=None,name_patt=None,super_cls=ROSStyleDefinitionParser):
    class ROSStyleMsgDefinitionParser(super_cls):
        if type_patt is not None:
            TYPE_PATT = type_patt
        if name_patt is not None:
            NAME_PATT = name_patt
        ALLOW_EMPTY_SEGMENTS = False
        SEGMENT_NAMES = ['msg']
    return ROSStyleMsgDefinitionParser

def get_ros_style_method_parser(type_patt=None,name_patt=None,allow_empty_segments=None,super_cls=ROSStyleDefinitionParser):
    class ROSStyleMethodDefinitionParser(super_cls):
        if type_patt is not None:
            TYPE_PATT = type_patt
        if name_patt is not None:
            NAME_PATT = name_patt
        if allow_empty_segments is not None:
            ALLOW_EMPTY_SEGMENTS = allow_empty_segments
        SEGMENT_NAMES = ['request','response']
    return ROSStyleMethodDefinitionParser

def get_ros_style_action_parser(type_patt=None,name_patt=None,super_cls=ROSStyleDefinitionParser):
    class ROSStyleActionDefinitionParser(super_cls):
        if type_patt is not None:
            TYPE_PATT = type_patt
        if name_patt is not None:
            NAME_PATT = name_patt
        SEGMENT_NAMES = ['goal','result','feedback']
    return ROSStyleActionDefinitionParser

class INISectionParser(DefFileParser.SectionParser):
    KEY_PATT = DefFileParser.BASIC_KEY_PATT
    VALUE_PATT = DefFileParser.BASIC_VALUE_PATT
    
    def parse(self):
        line_patt = DefFileParser.KEY_VALUE_PATT_TEMPLATE.format(key_patt=self.KEY_PATT,value_patt=self.VALUE_PATT)
        line_re = re.compile(line_patt)
        
        section = INISection(self.name)
        
        for line in self.reader():
            match = line_re.match(line)
            if not match:
                raise ParsingError('Parsing error: invalid INI section line\n{line}'.format(line=line))
            key = match.group('key')
            value = match.group('value')
            section.fields[key] = value
        return section

class ExtendedINISectionParser(INISectionParser):
    KEY_PATT = DefFileParser.BASIC_KEY_PATT
    VALUE_PATT = DefFileParser.BASIC_VALUE_PATT
    EXTENDED_VALUE_PATT = r'.*'
    
    def parse(self):
        line_patt = DefFileParser.KEY_VALUE_PATT_TEMPLATE.format(key_patt=self.KEY_PATT,value_patt=self.VALUE_PATT)
        line_re = re.compile(line_patt)
        
        extended_value_re = re.compile(self.EXTENDED_VALUE_PATT)
        
        section = INISection(self.name)
        
        extended_key = None
        extended_joiner = None
        extended_data = None
        for line in self.reader():
            #print line, extended_data
            match = line_re.match(line)
            if extended_key is not None:
                if match:
                    extended_data = extended_joiner.join(extended_data)
                    #print '1', extended_data
                    section.fields[extended_key] = extended_data
                    extended_key = None
                else:
                    extended_match = extended_value_re.match(line)
                    if not extended_match:
                        raise ParsingError('Parsing error: invalid INI section line\n{line}'.format(line=line))
                    extended_data.append(line)
                    continue
            if not match:
                raise ParsingError('Parsing error: invalid INI section line\n{line}'.format(line=line))
            key = match.group('key')
            value = match.group('value')
            if value == '|':
                extended_key = key
                extended_data = []
                extended_joiner = '\n'
            elif value == '>':
                extended_key = key
                extended_data = []
                extended_joiner = ' '
            else:
                extended_key = None
                section.fields[key] = value
        if extended_key is not None:
            extended_data = extended_joiner.join(extended_data)
            #print '2', extended_data
            section.fields[extended_key] = extended_data
        return section

def get_validated_INI_parser(key_patt,value_patt=None,format=None):
    class ValidatedINISectionParser(INISectionParser):
        if format:
            FORMAT = format
        KEY_PATT = key_patt
        if value_patt is not None:
            VALUE_PATT = value_patt
    return ValidatedINISectionParser

def get_validated_extended_INI_parser(key_patt,value_patt=None,extended_value_patt=None,format=None):
    class ValidatedINISectionParser(ExtendedINISectionParser):
        if format:
            FORMAT = format
        KEY_PATT = key_patt
        if value_patt is not None:
            VALUE_PATT = value_patt
        if extended_value_patt is not None:
            EXTENDED_VALUE_PATT = extended_value_patt
    return ValidatedINISectionParser

class RawSectionParser(DefFileParser.SectionParser):
    LINE_PATT = '.*'
    CONTENT_PATT = '.*'
    FOLD = False
    
    def parse(self):
        line_re = re.compile(self.LINE_PATT)
        
        section = RawSection(self.name)
        
        strs = ''
        
        for line in self.reader():
            match = line_re.match(line)
            if not match:
                raise ParsingError('Parsing error: invalid section line\n{line}'.format(line=line))
            strs.append(line)
        joiner = ' ' if self.FOLD else '\n'
        content = joiner.join(strs)
        
        if not re.match(self.CONTENT_PATT,content,flags=re.MULTILINE):
            raise ParsingError('Parsing error: invalid raw section\n{content}'.format(content=content))
        section.content = content
        
        return section

def get_validated_raw_section_parser(line_patt=None,content_patt=None,format=None):
    class ValidatedRawSectionParser(RawSectionParser):
        if format:
            FORMAT = format
        if line_patt:
            LINE_PATT = line_patt
        if content_patt:
            CONTENT_PATT = content_patt
    return ValidatedRawSectionParser

if __name__ == '__main__':
    data = \
"""[Manifest]
def-type = Application
name = my/application  

[msg:Image]
float val1
int val2

[srv:Reco]
Image image
----
String label
Pose pose

[Services]
reco = Reco
"""
    f = open('test.def','w')
    f.write(data)

"""[Manifest!ros]
def-type = ServiceCollection
name = my/application  

[msg:Image]
float val1
int val2

[srv:Reco]
Image image
----
String label
Pose pose

[Services]
reco_service = RecoService
"""

"""[Manifest]
def-type = Service
type = RecoService
name = reco_service

[msg:Image]
float val1
int val2

[method:Reco]
Image image
----
String label
Pose pose

[Methods]
reco = Reco
"""

"""[Manifest]
def-type = ros:ServiceCollection
type = RecoService
name = reco_service

[msg:Image]
float val1
int val2

[srv:Reco]
Image image
----
String label
Pose pose

[Service]
reco = Reco
"""