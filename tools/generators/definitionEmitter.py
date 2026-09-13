#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Writes objectDefinition.py and systemStructuresDefinition.py out in the NEW format:
#           real Python, under definitions/ at the repository root.
#
#           It does NOT parse anything. The two OLD generators in src/pythonGenerator/ already
#           build the intermediate representation while they run - parseInfo (the header keys)
#           plus parameterList (one dict per member) - and hand it here. So there is exactly one
#           parser for the old format and no second implementation to drift from it.
#
#           Collection is OFF unless EXUDYN_EMIT_DEFINITIONS is set, so a normal generator run -
#           and therefore tools/regenerate.py --check - is completely unaffected.
#
#           Revision plan step 31a. NOTHING reads the emitted files yet: they exist so the format
#           can be reviewed and changed cheaply before anything depends on it.
#
# Usage:    python tools/generators/emitDefinitions.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-13 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import copy
import io
import os
import re
import sys

emitDefinitions = (os.environ.get('EXUDYN_EMIT_DEFINITIONS', '0').strip().lower()
                   in ('1', 'true', 'yes', 'on'))

#definitions/ at the repository ROOT: the definitions are source data, not a tool, and the new
#generators live in tools/generators/ (revision plan step 31a)
outputDirectory = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                '..', '..', 'definitions'))

collectedDefinitions = []      #in FILE ORDER, which is load-bearing for byte-identical output

#---------------------------------------------------------------------------------- the legends
#THE EMITTER OWNS NO VOCABULARY. Every flag letter, named default, closed-set value and type
#constant comes from definitions/definitionTypes.py, which is hand-written and reviewed. Keeping
#a second copy here is what the old arrangement did, and two independent transcriptions of the
#same table drift with nothing to notice.
#
#The constants are grouped by NAME PREFIX, which is also how a reader tells them apart in the
#file itself; ConstantsWithPrefix turns a group back into the letter -> name map the emitter
#needs. A value the data uses but the module does not define is an ERROR naming what to add.
sys.path.insert(0, outputDirectory)
import definitionTypes


def ConstantsWithPrefix(prefix, skipPrefixes=()):
    """{value: constantName} for one prefix group, in declaration order. Raises when two
    constants in a group share a value, since then one of them could never round-trip."""
    byValue = {}
    for name in sorted(vars(definitionTypes).keys()):
        if not name.startswith(prefix) or name.startswith(skipPrefixes or ()):
            continue
        value = getattr(definitionTypes, name)
        if not isinstance(value, str):
            continue
        if value in byValue:
            raise ValueError('definitionTypes.py: ' + name + ' and ' + byValue[value]
                             + ' both stand for ' + repr(value) + ' - one could never be emitted')
        byValue[value] = name

    return byValue


itemDestinations = ConstantsWithPrefix('Dest')
itemFlags = ConstantsWithPrefix('CF')
structureFlags = ConstantsWithPrefix('SF')
namedDefaultValues = ConstantsWithPrefix('DV')
typeConstants = ConstantsWithPrefix('T')

#header keys whose value is one of a CLOSED set: a new value needs hand-written C++ as well, so a
#bare string in a definition file could only ever be a typo. Maps the key to its prefix group.
closedSetHeaderKeys = {
    'cParentClass':    ConstantsWithPrefix('ParentClass'),
    'mainParentClass': ConstantsWithPrefix('MainParentClass'),
    'visuParentClass': ConstantsWithPrefix('VisuParentClass'),
    'classType':       ConstantsWithPrefix('ClassType'),
    'objectType':      ConstantsWithPrefix('ObjectType'),
    }

#header keys whose value is the STRING "True"/"False" and becomes a real Python bool
booleanHeaderKeys = set(['writePybindIncludes', 'appendToFile', 'addDictionaryAccess',
                         'excludeFromTheDoc'])

#keys holding LaTeX, descriptions or C++ code: emitted as raw strings. Everything else is a short
#identifier-like value - measured: no member field except parameterDescription contains a
#backslash at all, and latexText contains real newlines but no literal backslash-n.
rawTextKeys = set(['classDescription', 'equations', 'miniExample', 'outputVariables', 'latexText',
                   'cppText', 'addConstructor', 'addProtectedC', 'addPublicC', 'addProtectedMain',
                   'addPublicMain', 'addIncludesC', 'addIncludesMain', 'author',
                   'parameterDescription'])

verbatimMultiLineKeys = set(['equations', 'miniExample'])

#which values the OLD parser mangled (a literal backslash-n became a real newline). It differs
#per generator, which is why this is a table. See UnmangleNewlines.
manglingRules = {
    'items':      {'mangleAll': True,  'keys': set(), 'verbatim': verbatimMultiLineKeys},
    'structures': {'mangleAll': False, 'verbatim': set(),
                   'keys': set(['classDescription', 'latexText', 'cppText'])},
    }

manglingSource = None

#---------------------------------------------------------------------------------- file layout
#the classType constants, not bare strings: this is the same closed set the definition
#files use, and it also decides which file a definition is emitted into
itemGroups = [('itemDefsNodes',   definitionTypes.ClassTypeNode),
              ('itemDefsObjects', definitionTypes.ClassTypeObject),
              ('itemDefsMarkers', definitionTypes.ClassTypeMarker),
              ('itemDefsLoads',   definitionTypes.ClassTypeLoad),
              ('itemDefsSensors', definitionTypes.ClassTypeSensor)]

structureGroups = [
    ('structureDefsSimulationSettings', [
        'SolutionSettings', 'NumericalDifferentiationSettings', 'DiscontinuousSettings',
        'NewtonSettings', 'GeneralizedAlphaSettings', 'ExplicitIntegrationSettings',
        'TimeIntegrationSettings', 'StaticSolverSettings', 'LinearSolverSettings', 'Parallel',
        'SimulationSettings']),
    ('structureDefsVisualizationSettings', [
        'VSettingsGeneral', 'VSettingsContourAdvanced', 'VSettingsContour', 'VSettingsNodes',
        'VSettingsBeams', 'VSettingsShells', 'VSettingsKinematicTree', 'VSettingsBodies',
        'VSettingsConnectors', 'VSettingsMarkers', 'VSettingsLoads', 'VSettingsTraces',
        'VSettingsSensors', 'VSettingsContact', 'VSettingsCamera', 'VSettingsScene',
        'VSettingsWindow', 'VSettingsView', 'VSettingsWindowDeprecated', 'VSettingsDialogs',
        'VSettingsMaterial', 'VSettingsRaytracerAdvanced', 'VSettingsRaytracer',
        'VSettingsOpenGLAdvanced', 'VSettingsLight', 'VSettingsOpenGL', 'VSettingsExportImages',
        'VSettingsOpenVR', 'VSettingsInteractiveAdvanced', 'VSettingsInteractive',
        'VisualizationSettings']),
    ('structureDefsSolverData', [
        'CSolverTimer', 'SolverLocalData', 'SolverIterationData', 'SolverConvergenceData',
        'SolverOutputData', 'SolverFileData']),
    ('structureDefsSolvers', [
        'MainSolverStatic', 'MainSolverImplicitSecondOrder', 'MainSolverExplicit']),
    ('structureDefsOther', [
        'PyBeamSection', 'BeamSectionGeometry']),
    ]

#every structure class, from the groups above - which WriteStructureDefinitions already checks
#both ways (a named class that was not parsed, and a parsed class in no group), so this is not a
#second list to maintain. A member whose TYPE is one of these names refers to the definition
#itself and needs no constant; the emitter checks the name exists, which is stronger than a
#constant, since a constant only verifies spelling.
structureClassNames = set()
for _, _classNames in structureGroups:
    structureClassNames.update(_classNames)

plainIdentifier = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def CollectDefinition(parseInfo, parameterList, lineDefinition, source):
    """Called by an old generator where one class block is complete, before it resets both."""
    global manglingSource
    if not emitDefinitions:
        return

    if source not in manglingRules:
        raise ValueError('unknown definition source: ' + str(source))
    manglingSource = source

    collectedDefinitions.append({'parseInfo': copy.deepcopy(parseInfo),
                                 'parameters': copy.deepcopy(parameterList),
                                 'lineDefinition': list(lineDefinition),
                                 'source': source})


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def UnmangleNewlines(text, key, source):
    """Restore the source spelling of a value.

    The old line parser turns EVERY literal backslash-n into a real newline. Intended as "enable
    line breaks", it also breaks the LaTeX macros that start with n - the two sources use
    backslash-nu, -nv, -nonumber, -noindent and -neq - each arriving as a newline followed by the
    rest of the macro name.

    Both cases are restorable and, better, they are DISTINGUISHABLE: a newline whose following
    text completes one of those five macro names at a macro boundary was a macro; anything else
    was a real line break. So macros get their backslash back and line breaks stay as newlines,
    which is what makes the emitted C++ snippets and multi-line descriptions readable.

    Either choice round-trips: a loader that reproduces the old parser applies
    text.replace(backslash-n, newline), and that maps both spellings back to the same value."""
    rule = manglingRules[source]
    if key in rule['verbatim'] or (not rule['mangleAll'] and key not in rule['keys']):
        return text

    out = []
    parts = text.split(chr(10))
    for index, part in enumerate(parts):
        if index > 0:
            out.append(chr(92) + 'n' if IsBrokenMacro(part) else chr(10))
        out.append(part)

    return ''.join(out)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the only LaTeX macros starting with n that the two sources use; verified against
#docs/theDoc/docincludes.sty (which defines nv) plus standard LaTeX
nMacroNames = ['nonumber', 'noindent', 'neq', 'nu', 'nv']


def IsBrokenMacro(textAfterNewline):
    """True when a newline plus this text is really a macro the parser split apart. The name must
    end at a macro boundary, so a line break before the word "using" is not read as the nu macro
    followed by "sing"."""
    for name in nMacroNames:
        rest = name[1:]                      #the leading n was consumed by the backslash-n
        if not textAfterNewline.startswith(rest):
            continue
        tail = textAfterNewline[len(rest):]
        if tail == '' or not tail[0].isalpha():
            return True

    return False


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def TypeConstantName(typeString):
    """T-constant name for a type, or None when the type is a C++ expression that has to stay a
    string. Constants cover the common, identifier-shaped types - which is 88% of item uses and
    98% of structure uses - and no more: naming 'template<class TReal> void' would produce a
    constant used once whose name is longer than the thing it names.

    An identifier-shaped type with no constant is an ERROR, not a fallback to a bare string: that
    is exactly the silent drift definitionTypes.py exists to prevent."""
    if not plainIdentifier.match(typeString or ''):
        return None

    #a structure name is not vocabulary - it points at a definition in these same files
    if typeString in structureClassNames:
        return None

    if typeString not in typeConstants:
        raise ValueError('no constant for type ' + repr(typeString)
                         + " - add T" + typeString + ' = ' + repr(typeString)
                         + ' to definitions/definitionTypes.py')

    return typeConstants[typeString]


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def StringLiteral(text, raw):
    """Raw only where the content can contain backslashes; everything else is an ordinary short
    literal, so the files do not carry r'' noise on every name and flag."""
    if text is None:
        return 'None'

    text = str(text)
    prefix = 'r' if raw else ''

    if chr(10) in text or (raw and (chr(92) in text and len(text) > 60)):
        if chr(34) * 3 in text or text.endswith(chr(92)) or text.endswith(chr(34)):
            return repr(text)
        return prefix + chr(34) * 3 + text + chr(34) * 3

    if not raw and chr(92) not in text:
        if "'" not in text:
            return "'" + text + "'"
        if chr(34) not in text:
            return chr(34) + text + chr(34)
        return repr(text)

    if text.endswith(chr(92)):
        return repr(text)
    if "'" not in text:
        return prefix + "'" + text + "'"
    if chr(34) not in text:
        return prefix + chr(34) + text + chr(34)

    return repr(text)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def FlagExpression(flagString, table):
    """'CDI' -> CFConst+CFDeclarationOnly+CFInterface; the order of the original string is kept so
    that a round-trip is exact."""
    if not flagString:
        return "''"

    parts = []
    for letter in flagString:
        if letter not in table:
            raise ValueError('undocumented flag letter ' + repr(letter) + ' in '
                             + repr(flagString) + ' - add it to definitions/definitionTypes.py')
        parts.append(table[letter])

    return '+'.join(parts)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def DefaultValueExpression(value):
    if value == '':
        return None
    if value in namedDefaultValues:
        return namedDefaultValues[value]

    return StringLiteral(value, raw=(chr(92) in value))


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def TypeExpression(typeString):
    name = TypeConstantName(typeString)
    if name is not None:
        return name

    return StringLiteral(typeString, raw=False)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def EmitMember(parameter, source):
    """One member as a constructor call. type / destination / flags go on the FIRST line, because
    together they classify the member; everything else follows one per line so that a changed
    description stays a one-line diff."""
    lineType = parameter.get('lineType', '')
    isFunction = lineType.startswith('F')
    suffix = lineType[1:]
    isVirtual = 'v' in suffix
    isStatic = 's' in suffix
    isLinked = 'L' in suffix
    fromParent = 'p' in suffix

    if source == 'items':
        constructor = 'ItemFunction' if isFunction else 'ItemParameter'
        flagTable = itemFlags
    else:
        constructor = 'StructureFunction' if isFunction else 'StructureParameter'
        flagTable = structureFlags

    typeString = parameter.get('type', '')
    cFlags = parameter.get('cFlags', '') or ''

    #SFSubstructure is DERIVED, not written: for a structure it says exactly "the type is one of
    #the classes defined here", which holds for all 71 of them. Writing it again is a second
    #statement of the same fact, and two statements can disagree. The assertion below is what a
    #future disagreement hits instead of the reader.
    if source == 'structures':
        isSubstructure = typeString in structureClassNames
        if isSubstructure != ('S' in cFlags):
            raise ValueError(parameter.get('pythonName', '?') + ': type ' + repr(typeString)
                             + (' names a structure but has no S flag' if isSubstructure
                                else ' has the S flag but is not a structure defined here'))
        cFlags = cFlags.replace('S', '')

    head = ['type=' + TypeExpression(typeString)]
    if source == 'items':
        head.append('destination=' + FlagExpression(parameter.get('destination', ''),
                                                    itemDestinations))
    head.append('cFlags=' + FlagExpression(cFlags, flagTable))
    for name, active in (('isVirtual', isVirtual), ('isStatic', isStatic),
                         ('isLinked', isLinked), ('fromParent', fromParent)):
        if active:
            head.append(name + '=True')

    lines = ['        ' + constructor + '(' + ', '.join(head) + ',']

    body = [('pythonName', StringLiteral(parameter.get('pythonName', ''), raw=False))]

    #cplusplusName is omitted when it equals pythonName: the old file left it empty and the parser
    #filled it in, so omitting restores the intent rather than the intermediate
    cpp = parameter.get('cplusplusName', '')
    if cpp and cpp != parameter.get('pythonName', ''):
        body.append(('cplusplusName', StringLiteral(cpp, raw=False)))

    if parameter.get('size', ''):
        body.append(('size', StringLiteral(parameter['size'], raw=False)))

    if isFunction:
        if parameter.get('args', ''):
            body.append(('args', StringLiteral(parameter['args'], raw=False)))
        if parameter.get('defaultValue', ''):
            body.append(('implementation', DefaultValueExpression(parameter['defaultValue'])))
    else:
        default = DefaultValueExpression(parameter.get('defaultValue', ''))
        if default is not None:
            body.append(('defaultValue', default))
        if parameter.get('args', ''):
            body.append(('args', StringLiteral(parameter['args'], raw=False)))

    description = parameter.get('parameterDescription', '')
    if description:
        text = UnmangleNewlines(str(description), 'parameterDescription', source)
        body.append(('description', StringLiteral(text, raw=True)))

    for key, expression in body:
        lines.append('            ' + key + '=' + expression + ',')
    lines[-1] = lines[-1][:-1] + '),'

    return '\n'.join(lines)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bannerWidth = 99        #'#' plus 98 '+', the banner width used throughout the project


def Banner(className):
    """A three-line separator carrying the class name, so a definition can be found by eye in a
    file that is several hundred kilobytes long."""
    rule = '#' + '+' * (bannerWidth - 1)
    middle = '#' + '+' * 16 + '   ' + className + '   '

    return [rule, middle + '+' * max(3, bannerWidth - len(middle)), rule]


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def EmitDefinition(definition):
    parseInfo = definition['parseInfo']
    source = definition['source']
    constructor = 'ItemDefinition' if source == 'items' else 'StructureDefinition'

    className = parseInfo.get('class', '')
    lines = Banner(className)
    lines.append('definitions.append(' + constructor + '(')
    lines.append('    className=' + StringLiteral(className, raw=False) + ',')

    for key in sorted(parseInfo.keys()):
        if key in ('class', 'writeFile'):
            continue
        value = parseInfo[key]
        if value is None or value == '':
            continue
        if key in booleanHeaderKeys:
            lines.append('    ' + key + '=' + ('True' if value == 'True' else 'False') + ',')
            continue
        if key in closedSetHeaderKeys:
            table = closedSetHeaderKeys[key]
            if value not in table:
                raise ValueError(className + ': ' + key + ' = ' + repr(value)
                                 + ' is not a known value - if this is really a new one, add a'
                                 + ' constant to definitions/definitionTypes.py (a new '
                                 + key + ' also needs hand-written C++)')
            lines.append('    ' + key + '=' + table[value] + ',')
            continue
        text = UnmangleNewlines(str(value), key, source)
        lines.append('    ' + key + '=' + StringLiteral(text, raw=(key in rawTextKeys)) + ',')

    lines.append('    members=[')
    for parameter in definition['parameters']:
        lines.append(EmitMember(parameter, source))
    lines.append('        ],')
    lines.append('    ))')
    lines.append('')

    return '\n'.join(lines)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def WriteGroup(moduleName, title, sourceFile, definitions):
    Q = chr(39)
    classNames = [d['parseInfo'].get('class', '') for d in definitions]
    L = []
    L.append('#' + '+' * 98)
    L.append('# ' + title)
    L.append('#')
    L.append('# Details:  ' + str(len(classNames)) + ' definitions, emitted from ' + sourceFile
             + ' (revision plan step 31a).')
    L.append('#           This IS Python: import it and read "definitions", a list of dicts.')
    L.append('#')
    L.append('#           ORDER MATTERS. The generators emit in the order the definitions appear,')
    L.append('#           and the generated C++/pybind/RST is compared byte-for-byte, so')
    L.append('#           reordering this list changes generated files. Append at the end unless')
    L.append('#           you mean to reorder.')
    L.append('#')
    L.append('#           Only descriptions, LaTeX and C++ code are raw strings; every other field')
    L.append('#           is a name, a flag constant or a short literal and needs no escaping.')
    L.append('#')
    L.append('#           The constants come from definitionTypes.py, which is hand-written: a')
    L.append('#           value used here with no constant there stops the emit and says what to')
    L.append('#           add, so the two can never drift apart silently.')
    L.append('#')
    L.append('# Contents: ' + ', '.join(classNames[:6]) + ('' if len(classNames) <= 6 else ', ...'))
    L.append('#')
    L.append('# Copyright:This file is part of Exudyn. Exudyn is free software: see '
             + Q + 'LICENSE.txt' + Q)
    L.append('#' + '+' * 98)
    L.append('')
    L.append('from definitionTypes import *')
    L.append('')
    L.append('definitions = []')
    L.append('')

    text = '\n'.join(L) + '\n'.join([EmitDefinition(d) for d in definitions])
    path = os.path.join(outputDirectory, moduleName + '.py')
    io.open(path, 'w', encoding='utf8', newline='\n').write(text)
    print('  %-42s %3d definitions, %8d bytes' % (moduleName + '.py', len(definitions), len(text)))


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def WriteItemDefinitions():
    if not emitDefinitions:
        return
    if not os.path.isdir(outputDirectory):
        os.makedirs(outputDirectory)

    print('emitting item definitions (' + str(len(collectedDefinitions)) + ' items):')
    covered = 0
    for moduleName, category in itemGroups:
        group = [d for d in collectedDefinitions
                 if d['parseInfo'].get('classType', '') == category]
        covered += len(group)
        WriteGroup(moduleName, category + ' item definitions', 'objectDefinition.py', group)

    if covered != len(collectedDefinitions):
        raise ValueError('emitted %d of %d items - a classType is outside the five categories'
                         % (covered, len(collectedDefinitions)))


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def WriteStructureDefinitions():
    if not emitDefinitions:
        return
    if not os.path.isdir(outputDirectory):
        os.makedirs(outputDirectory)

    present = set([d['parseInfo'].get('class', '') for d in collectedDefinitions])
    print('emitting structure definitions (' + str(len(collectedDefinitions)) + ' structures):')
    assigned = set()
    for moduleName, classNames in structureGroups:
        missing = [n for n in classNames if n not in present]
        if missing:
            raise ValueError('group "' + moduleName + '" names classes that were not parsed: '
                             + ', '.join(missing))
        assigned.update(classNames)
        wanted = set(classNames)
        group = [d for d in collectedDefinitions if d['parseInfo'].get('class', '') in wanted]
        WriteGroup(moduleName, moduleName[len('structureDefs'):] + ' definitions',
                   'systemStructuresDefinition.py', group)

    unassigned = sorted(present - assigned)
    if unassigned:
        raise ValueError('structures in no group: ' + ', '.join(unassigned)
                         + '\n  add them to structureGroups in definitionEmitter.py')
