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
import outputVariableDescriptions
import outputVariableTypes


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
#the named composite defaults are CppValue objects now, not strings, so they are collected by
#their C++ rendering - which is what the old format stored
namedDefaultValues = {}
for _name in sorted(vars(definitionTypes).keys()):
    if _name.startswith('DV'):
        _value = getattr(definitionTypes, _name)
        if isinstance(_value, definitionTypes.CppValue):
            if _value.ToCpp() in namedDefaultValues:
                raise ValueError('definitionTypes.py: ' + _name + ' and '
                                 + namedDefaultValues[_value.ToCpp()] + ' both stand for '
                                 + repr(_value.ToCpp()) + ' - one could never be emitted')
            namedDefaultValues[_value.ToCpp()] = _name
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
def NumberExpression(value, typeName):
    """The Python literal for a C++ numeric default, or None if it is not a number. The literal is
    the C++ spelling with the f suffix removed, which is always valid Python and keeps the
    definition file reading exactly like the value it stands for (0., -1., 1e-8, 0.001).

    It is emitted ONLY if it renders back to the original text. So the check is not a rule that
    has to be trusted: a spelling the formatter cannot reproduce stays a string and is visible.
    """
    #surrounding whitespace is part of the stored value (VSettingsTraces.triadSize is '0.1f '
    #with a trailing space), and a number cannot carry it, so such a value stays a string
    if value != value.strip() or not value:
        return None
    text = value
    core = text[:-1] if text.endswith('f') and not text.endswith('lf') else text

    try:
        number = int(core)
    except ValueError:
        try:
            number = float(core)
        except ValueError:
            return None

    if definitionTypes.CppLiteral(number, typeName) != text:
        return None

    return core


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def DefaultValueExpression(value, typeName=''):
    """A default value as a real Python value wherever it is one: True/False for the C++ literals,
    a number for a number, a named constant for the composite ones, a string for the rest (C++
    constructor calls such as Vector3D({1.,0.,0.}), which are code, not values)."""
    if value == '':
        return None
    if value in namedDefaultValues:
        return namedDefaultValues[value]
    if value == 'true':
        return 'True'
    if value == 'false':
        return 'False'

    number = NumberExpression(value, typeName)
    if number is not None:
        return number

    return StringLiteral(value, raw=(chr(92) in value))


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the families that definitionTypes.py folds into constructors. Built by INVERTING the tables in
#that module, so the emitter still states nothing of its own: if a size is added there, it is
#emittable here on the next run without touching this file.
rangeForms = {}
for _base in ('TReal', 'Tfloat', 'TIndex'):
    for _key, _name in getattr(definitionTypes, _base).constrainedForms.items():
        _argument = ({'minimum': 'minimum=0', 'greaterThan': 'greaterThan=0'}
                     .get(_key, 'Item' + _key))
        rangeForms[_name] = _base + '(' + _argument + ')'

arrayForms = {'ArrayIndex': 'TArrayIndex'}
for _key, _name in definitionTypes.TArrayIndex.constrainedForms.items():
    arrayForms[_name] = 'TArrayIndex(Item' + _key + ')'

shapeForms = {}
for _n, _name in definitionTypes.vectorSizes.items():
    shapeForms[_name] = ('TVectorND(' + str(_n) + ')', str(_n))
for (_r, _c), _name in definitionTypes.matrixSizes.items():
    shapeForms[_name] = ('TMatrixND(' + str(_r) + ', ' + str(_c) + ')', str(_r * _c))
for _n, _name in definitionTypes.indexTupleSizes.items():
    shapeForms[_name] = ('TIndexND(' + str(_n) + ')', str(_n))
for _n, _name in definitionTypes.nodeIndexTupleSizes.items():
    shapeForms[_name] = ('TIndexND(' + str(_n) + ', ItemNode)', str(_n))

#Float3/Float4 and StdArray33F are fixed-shape but have no ND family of their own
fixedShapes = {'Float3': '3', 'Float4': '4', 'StdArray33F': '3x3', 'Matrix6D': '36'}

#a variable-length container: the old format wrote size=-1 to say "any length"
variableLength = set(['Vector', 'NumpyVector', 'NumpyMatrix', 'ArrayFloat'] + list(arrayForms))

#three parameters declared Vector6D with size=3, which contradicts their own type, default value,
#description and GetNumberOfODE2Coordinates - a published documentation error (#2410). The new
#form cannot express the contradiction, so the stale size is dropped here.
knownWrongSizes = set([('NodeRigidBodyRotVecLG', 'referenceCoordinates'),
                       ('NodeRigidBodyRotVecLG', 'initialCoordinates'),
                       ('NodeRigidBodyRotVecLG', 'initialVelocities')])


def TypeExpression(typeString, size='', isFunction=False, owner=''):
    """The type as a constructor call carrying its shape and range, or a plain name.

    Returns (expression, leftoverSize). leftoverSize is always '' today: every declared size is
    either implied by the type or becomes an argument of it - measured over all 338 members that
    carry one. It is returned rather than asserted away so that a future size the type cannot
    express is reported instead of silently lost."""
    size = (size or '').strip()

    #a function's size was never meaningful - the legend says size is "used for variables and
    #vectors and matrices only", and the 4 that carry one are all 'void' with a size
    if isFunction:
        size = ''

    if typeString in shapeForms:
        expression, implied = shapeForms[typeString]
        if size and size != implied and (owner not in knownWrongSizes):
            raise ValueError(str(owner) + ': type ' + typeString + ' implies size ' + implied
                             + ' but size=' + size + ' is declared')
        return expression, ''

    if typeString in rangeForms:
        return rangeForms[typeString], ''

    if typeString in arrayForms:
        #a real length constraint (2 markers on a connector, 3 constrained axes) belongs in the
        #type. -1 is kept explicit rather than treated as "absent": the structures dialog turns an
        #absent size into {1}, so dropping it would change what the dialog reports.
        argument = '' if size == '' else 'size=' + size
        expression = arrayForms[typeString]
        if argument:
            expression = (expression + '(' + argument + ')' if '(' not in expression
                          else expression[:-1] + ', ' + argument + ')')
        return expression, ''

    if size:
        if typeString in fixedShapes and size == fixedShapes[typeString]:
            size = ''                      #implied by the type
        elif size == '1':
            #a scalar: says nothing, and the structures dialog emits {1} for an absent size anyway
            size = ''
        #size=-1 ("any length") on a container that is not in the ArrayIndex family falls through
        #to leftoverSize and stays written, because dropping it would make the dialog report {1}

    name = TypeConstantName(typeString)
    if name is not None:
        return name, size

    return StringLiteral(typeString, raw=False), size


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the shared function declarations. A member that matches one is emitted as ItemFunctionDef(...)
#and the declaration is not restated; anything else is written out in full.
import itemFunctions

functionLibrary = itemFunctions.itemFunctionLibrary


def VisibleEntries(pythonName, classType, parentClass):
    """The entries a use site in this class can reach - the same wildcard rule the resolver in
    definitionTypes.py applies: an entry with classType or parentClass None fits every class."""
    return [e for e in functionLibrary
            if e['pythonName'] == pythonName
            and (e['classType'] is None or e['classType'] == classType)
            and (e['parentClass'] is None or e['parentClass'] == parentClass)]


def LibraryEntry(parameter, classType, parentClass, cleanedFlags):
    """The single library entry this member stands for, or None."""
    candidates = [e for e in VisibleEntries(parameter.get('pythonName', ''),
                                            classType, parentClass)
                  if str(e['type']) == str(parameter.get('type', ''))
                  and e['destination'] == str(parameter.get('destination', ''))
                  and e['cFlags'] == cleanedFlags
                  and e['args'] == (parameter.get('args', '') or '')]
    if len(candidates) != 1:
        return None

    return candidates[0]


def DisambiguatingFields(entry, classType, parentClass):
    """Which of destination/cFlags/args the use site has to give, because the name alone does not
    identify the entry. Only the fields that actually differ are named, so the call stays short."""
    same = VisibleEntries(entry['pythonName'], classType, parentClass)
    if len(same) < 2:
        return []

    return [field for field in ('destination', 'cFlags', 'args')
            if len(set(str(e[field]) for e in same)) > 1]


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def EmitMember(parameter, source, className=chr(39)+chr(39), classType='', parentClass=''):
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

    #the interface flag is INVERTED for parameters and DROPPED for functions. Inverted because
    #947 of 985 parameters are in the Python interface and 38 are not, so the exception is what
    #is worth writing down. Dropped for functions because it has no effect there: the generator
    #reads it only inside a block guarded by lineType 'V'
    #(pythonAutoGenerateObjects.py:1123), and 1755 of 1850 function rows carried it anyway.
    #'declaration only' is DERIVED: a function with no implementation is a declaration, one with
    #an implementation gets a body, and an EMPTY implementation is an empty body - three states
    #that a flag beside a text field could only ever restate. 53 item functions carried the flag
    #AND an implementation of ';', which the generator ignored: dead text, dropped here.
    if isFunction:
        cFlags = cFlags.replace('D', '')

    if source == 'items':
        if isFunction:
            cFlags = cFlags.replace('I', '')
        elif 'I' in cFlags:
            cFlags = cFlags.replace('I', '')
        else:
            cFlags = cFlags + 'n'

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

    typeExpression, leftoverSize = TypeExpression(
        typeString, parameter.get('size', ''), isFunction,
        (className, parameter.get('pythonName', '')))
    head = ['type=' + typeExpression]
    if source == 'items':
        head.append('destination=' + FlagExpression(parameter.get('destination', ''),
                                                    itemDestinations))
    #an empty flag set is written as nothing at all: with the interface flag inverted, most
    #parameters carry no flag, and 'cFlags=' + two quotes is noise
    flagExpression = FlagExpression(cFlags, flagTable)
    if flagExpression != chr(39) * 2:
        head.append('cFlags=' + flagExpression)
    #isVirtual defaults to True, so it is written only for the minority that do NOT override a
    #parent function - 195 of 1850 item functions
    if isFunction and not isVirtual:
        head.append('isVirtual=False')
    for name, active in (('isStatic', isStatic),
                         ('isLinked', isLinked), ('fromParent', fromParent)):
        if active:
            head.append(name + '=True')

    entry = None
    if source == 'items' and isFunction and isVirtual and not isStatic:
        entry = LibraryEntry(parameter, classType, parentClass, cFlags)
    if entry is not None:
        short = ['        ItemFunctionDef(' + StringLiteral(parameter['pythonName'], raw=False)]
        for field in DisambiguatingFields(entry, classType, parentClass):
            if field == 'destination':
                short.append('destination='
                             + FlagExpression(entry['destination'], itemDestinations))
            elif field == 'cFlags':
                short.append('cFlags=' + (FlagExpression(entry['cFlags'], itemFlags)
                                          if entry['cFlags'] else chr(39) * 2))
            else:
                short.append('args=' + StringLiteral(entry['args'], raw=False))
        implementation = parameter.get('defaultValue', '')
        if 'D' not in (parameter.get('cFlags', '') or ''):
            implementation = implementation or ''
            short.append('implementation='
                         + StringLiteral(implementation, raw=(chr(92) in implementation)))
        description = parameter.get('parameterDescription', '')
        if description != entry['description']:
            short.append('description='
                         + StringLiteral(description, raw=(chr(92) in description)))
        if parameter.get('cplusplusName', '') != parameter.get('pythonName', ''):
            short.append('cplusplusName='
                         + StringLiteral(parameter.get('cplusplusName', ''), raw=False))

        return '        ItemFunctionDef(' + (',\n            '
                                             .join([StringLiteral(parameter['pythonName'],
                                                                  raw=False)]
                                                   + short[1:])) + '),'

    lines = ['        ' + constructor + '(' + ', '.join(head) + ',']

    body = [('pythonName', StringLiteral(parameter.get('pythonName', ''), raw=False))]

    #cplusplusName is omitted when it equals pythonName: the old file left it empty and the parser
    #filled it in, so omitting restores the intent rather than the intermediate
    cpp = parameter.get('cplusplusName', '')
    if cpp and cpp != parameter.get('pythonName', ''):
        body.append(('cplusplusName', StringLiteral(cpp, raw=False)))

    if leftoverSize:
        #nothing reaches this today; it exists so a size the type cannot express is written out
        #and visible, rather than dropped
        body.append(('size', StringLiteral(leftoverSize, raw=False)))

    if isFunction:
        if parameter.get('args', ''):
            body.append(('args', StringLiteral(parameter['args'], raw=False)))
        #a function's 'defaultValue' is its C++ body. None means "declaration only", which the
        #old format said with the D flag; '' means an empty body, which 98 functions have.
        if 'D' in (parameter.get('cFlags', '') or ''):
            pass                                   #declaration only: implementation stays None
        else:
            implementation = parameter.get('defaultValue', '') or ''
            body.append(('implementation',
                         StringLiteral(implementation, raw=(chr(92) in implementation))))
    else:
        #a deprecated member has no default value, so the old format stored the deprecation
        #version and expiry year there as 'version;EXP=year'. That is two facts in one string and
        #it is not a default, so it gets its own field.
        rawDefault = parameter.get('defaultValue', '')
        if 'X' in cFlags and ';EXP=' in str(rawDefault):
            since, expires = str(rawDefault).split(';EXP=', 1)
            body.append(('deprecated',
                         'Deprecated(' + StringLiteral(since, raw=False) + ', ' + expires + ')'))
            rawDefault = ''
        default = DefaultValueExpression(rawDefault, typeString)
        body.append(('defaultValue', default if default is not None else 'NoDefaultValue'))
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
#the description texts several items share verbatim, inverted so the emitter can name them
sharedDescriptions = {}
for _name in dir(outputVariableDescriptions):
    if _name.startswith('OVD'):
        sharedDescriptions[getattr(outputVariableDescriptions, _name)] = _name

knownOutputVariables = set(v.name for v in outputVariableTypes.outputVariableTypes)


def EmitOutputVariables(text, className):
    """The old format stores the output variables as ONE string holding a Python dict literal,
    which both generators then eval(). Here it becomes a list of real entries: the key is a
    constant, so a typo is a NameError instead of a variable that silently never matches, and
    nothing has to eval() a string that was assembled with backslash gymnastics."""
    table = eval(text.replace(chr(10), chr(92) + 'n').replace(chr(92), chr(92) * 2))

    lines = ['    outputVariables=[']
    for key, description in table.items():
        if key not in knownOutputVariables:
            raise ValueError(className + ': output variable ' + repr(key) + ' is not declared in'
                             + ' definitions/outputVariableTypes.py')
        if description in sharedDescriptions:
            value = sharedDescriptions[description]
        else:
            value = StringLiteral(description, raw=(chr(92) in description))
        lines.append('        ItemOutputVariable(OV' + key + ', ' + value + '),')
    lines.append('        ],')

    return lines


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
        if key == 'outputVariables':
            lines += EmitOutputVariables(text, className)
            continue
        lines.append('    ' + key + '=' + StringLiteral(text, raw=(key in rawTextKeys)) + ',')

    lines.append('    members=[')
    for parameter in definition['parameters']:
        lines.append(EmitMember(parameter, source, parseInfo.get("class", ""),
                                parseInfo.get('classType', ''),
                                parseInfo.get('cParentClass', '')))
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
    if sourceFile == 'objectDefinition.py':
        #OV... names the output variables, OVD... the descriptions shared by several items
        L.append('from outputVariableTypes import *')
        L.append('from outputVariableDescriptions import *')
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
