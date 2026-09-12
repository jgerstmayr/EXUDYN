# -*- coding: utf-8 -*-
"""
Created on Tue Feb 03 15:00:30 2026

@author: Johannes Gerstmayr

goal: convert old comments into docstrings
"""


import argparse
import re
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Any


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#remove indentation of text block (for rst files) and afterwards add specific indentation:
def RemoveIndentation(text, addSpaces = '', removeAllSpaces = True, removeIndentation = True):
    lines=text.replace('\t','    ').split('\n')
    s = ''
    hasEndl = False
    if lines[-1] == '':
        hasEndl = True
        del lines[-1]
    
    if not removeAllSpaces:
        minIndent = 10000
        for line in lines:
            if line != '':
                nSpaces = len(line)-len(line.lstrip(' '))
                minIndent=min(minIndent, nSpaces)
        
        if removeIndentation:
            for i, line in enumerate(lines):
                lines[i] = line[minIndent:]
    else:
        for i, line in enumerate(lines):
            lines[i] = line.lstrip()
        
    for i, line in enumerate(lines):
        s += addSpaces+line
        if i < len(lines)-1:
            s += '\n'

    if hasEndl: 
        s+='\n' #in this case, we had an endline and like to keep it

    return s #omit last \n


#for text in .pyi (stub) files, some characters and strings make problems
def CleanStringForPyiDescription(text, enforcePeriod=False):
    if len(text) == 0: 
        return text

    #text = text.replace(r'\codeName','Exudyn').replace(r'$\ra$','$\rightarrow$').replace(r'\tabnewline', '\n') #→
    text = text.replace(r'\codeName','Exudyn').replace(r'$\ra$','→').replace(r'\tabnewline', '\n') #
    text = text.replace(r'\tnu',r'\nu')
    #text = text.replace(r'\be',r'$').replace(r'\ee',r'$') #rarely occurs
    
    text = text.replace(r'\pv',r'\mathbf{p}').replace(r'\tv',r'\mathbf{t}').replace(r'\uv',r'\mathbf{u}')
    text = text.replace(r'\pLocB',r'{}^{b}{\mathbf{b}}')
    #text = text.replace(r'\Am',r'\mathbf{A}')
    
    text = re.sub(r'\\texttt\{(.*?)\}', r'``\1``', text) #NOTE: regex needs r"\\" - two backslash! 

    #special replacements:    
    pattern = r'\\LU\{([^}]*)\}\{([^}]*)\}' #for "\\LU{text1}{text2}"
    #text = re.sub(pattern, r'\\prescript{\1}{}{\2}', text) #\prescript not available with docstrings
    text = re.sub(pattern, r'{}^{\1}{\2}', text) #workaround
    

    
    text = re.sub(r'\\refSection\{(.*?)\}', 'theDoc.pdf', text) #eliminate
    text = re.sub(r'\\acp\{(.*?)\}', r'\1', text) #convert to readable text
    text = re.sub(r'\\ac\{(.*?)\}', r'\1', text) #convert to readable text
    text = re.sub(r'\\hac\{(.*?)\}', r'\1', text) #convert to readable text
    text = text.replace(r'\_','_')

    #++++++++++++++++++++++
    #distinguish between latex and other parts
    parts = re.split(r'(\$\$.*?\$\$|\$.*?\$)', text, flags=re.DOTALL)
    
    cleaned_parts = []
    for part in parts:
        if part.startswith('$'):
            # This is a math block; keep it exactly as is
            cleaned_parts.append(part)
            # if '3D rigid body node' in text:
            #     print('\n$part=',part)
        else:
            # if '3D rigid body node' in text:
            #     print('\npart =',part)
            # This is regular text; remove backslashes
            cleaned_parts.append(part.replace('\\', '').replace('"',"'").replace('}{','} {'))
            
    text =  "".join(cleaned_parts)
    #++++++++++++++++++++++

    inline_pattern = r"\$([^\$]+?)\$" #formulas to reST
    text = re.sub(inline_pattern, r":math:`\1`", text)
    
    if "\\texttt" in text or "\\ac" in text:
        print('WARNING: "\\texttt" in text or "\\ac" in text:\n'+text)

    text = text.rstrip()
    if enforcePeriod and len(text) and text[-1] not in ['.','?','!']:
        text = text.rstrip()+'.' #avoid warnings

    return text

def SplitSummaryDescription(text):
    idx = text.find(". ")
    if idx != -1 and text[:idx+1].endswith('e.g.'):
        idx = text.find(". ",idx+1)

    description = ''
    if idx == -1:
        # No period -> entire text is summary; ensure it ends with a period.
        summary = text
        if len(summary.rstrip()) and summary.rstrip()[-1] not in ['.','?','!']:
            summary = text.rstrip()+'.' #avoid warnings
    else:
        # Split into summary (including the '.') and description
        summary = text[:idx].strip() + '.'
        description = text[idx+2:].strip()
    return (summary, description)

#convert a simple description for function, parameter or class into docstring
#NOTE: latex formulas are purely rendered, therefore "\" is converted to "\\" and "}{" to "} {"
def DocStringGoogleFromPlainText(text, addSpaces='    ', 
                                 multiline=True, 
                                 splitSummaryDescription=False, 
                                 endLine=True):
    """Convert text into docstring.

    Args:
        text (str): input text.
        addSpaces (TYPE, optional): spaces added in front. Defaults to '    '.
        multiline (TYPE, optional): if False, uses only single line. Defaults to True.
        splitSummaryDescription (TYPE, optional): if True, splits text into summary (until first period) and remaining text. Defaults to False.
        endLine (TYPE, optional): if True, addes line break at end. Defaults to True.

    Returns:
        finalText (str): converted docstring.
    """
    text = CleanStringForPyiDescription(text, enforcePeriod=not splitSummaryDescription) #for single strings=>summary, needs period!
    addR = 'r' if '\\' in text else ''

    summary = text
    description = ""

    if splitSummaryDescription and multiline:
        (summary, description) = SplitSummaryDescription(text)
        # idx = text.find(".")
        # if idx == -1:
        #     # No period -> entire text is summary; ensure it ends with a period.
        #     summary = text
        #     if len(summary) and summary[-1] not in ['.','?','!']:
        #         summary = text.rstrip()+'.' #avoid warnings
        # else:
        #     # Split into summary (including the '.') and description
        #     summary = text[:idx + 1].strip() + '.'
        #     description = text[idx + 1:].strip()

    finalText = ''
    if multiline and description!='':
        finalText += addSpaces+addR+'"""'
        if splitSummaryDescription:
            finalText += summary+"\n"
            text = description

        text = RemoveIndentation(text.replace('. ','\n'), addSpaces = addSpaces, 
                                 removeAllSpaces = False, 
                                 removeIndentation = True)
        finalText += "\n"+text+"\n"+addSpaces+'"""'+'\n'*endLine
    else:
        text = RemoveIndentation(text).replace('\n','; ').strip() #to keep line breaks visible
        finalText += addSpaces+addR+'"""'+text+'"""'
        finalText+='\n'*endLine 
    
    return finalText



# ----------------------------
# Data structures
# ----------------------------


@dataclass
class Param:
    name: str
    type_hint: Optional[str] = None
    description: str = ""

@dataclass
class OutputSpec:
    type_hint: Optional[str] = None
    description: str = ""

@dataclass
class DocBlock:
    kind: Optional[str] = None  # 'function', 'class', 'classFunction'
    summary: str = ""
    inputs: List[Param] = field(default_factory=list)
    output: Optional[OutputSpec] = None
    notes: List[str] = field(default_factory=list)
    examples: List[str] = field(default_factory=list)
    author: Optional[str] = None
    date: Optional[str] = None
    belongs_to: Optional[str] = None

    start_line: Optional[int] = None
    end_line: Optional[int] = None

    def is_complete(self) -> bool:
        return self.kind is not None and self.end_line is not None

    def to_dict(self) -> Dict[str, Any]:
        """Export the DocBlock content into a serializable dictionary."""
        return {
            "kind": self.kind,
            "summary": self.summary,
            "inputs": [
                {"name": p.name, "type_hint": p.type_hint, "description": p.description}
                for p in self.inputs
            ],
            "output": (
                {"type_hint": self.output.type_hint, "description": self.output.description}
                if self.output is not None else None
            ),
            "notes": list(self.notes),
            "examples": list(self.examples),
            "author": self.author,
            "date": self.date,
            "belongs_to": self.belongs_to,
        }

# ----------------------------
# Parsing helpers
# ----------------------------

TAG_PATTERN = re.compile(r"^\s*#\*\*(\w+)\s*:(.*)$")
DEF_PATTERN = re.compile(r"^\s*def\s+([A-Za-z_]\w*)\s*\(")
CLASS_PATTERN = re.compile(r"^\s*class\s+([A-Za-z_]\w*)\s*[\(:]")
COMMENT_LINE_PATTERN = re.compile(r"^\s*#(.*)$")

# Set of recognized tags
RECOGNIZED_TAGS = {
    "class",
    "function",
    "classFunction",
    "input",
    "output",
    "author",
    "date",
    "notes",
    "example",
    "status",      # will be ignored in rendering
    "belongsTo",
}

# ----------------------------
# Tag content processors
# ----------------------------

def parse_input_line(raw: str) -> Optional[Param]:
    """
    Parse a single input line of the form:
      '#  name: description'
    Returns Param or None if not parseable.
    """
    # Strip leading '#' and whitespace
    m = COMMENT_LINE_PATTERN.match(raw)
    if not m:
        return None
    content = m.group(1).strip()
    if not content:
        return None
    # Split at first colon
    parts = content.split(":", 1)
    if len(parts) < 2:
        # no colon -> cannot parse param
        return None
    name = parts[0].strip()
    desc = parts[1].strip()
    desc = CleanStringForPyiDescription(desc)
    if not name:
        return None
    return Param(name=name, description=desc)

def parse_notes_line(raw: str) -> Optional[str]:
    m = COMMENT_LINE_PATTERN.match(raw)
    if not m:
        return None
    return CleanStringForPyiDescription(m.group(1).rstrip())

def parse_examples_line(raw: str) -> Optional[str]:
    m = COMMENT_LINE_PATTERN.match(raw)
    if not m:
        return None
    return m.group(1).rstrip()

def parse_output_header(initial_content: str) -> Tuple[Optional[str], str]:
    """
    Parse the header of an output tag after '#**output:'.
    Output types are encapsulated in colons, e.g., ':int:' or ':Union[dict, ObjectIndex]:'.
    The header may also contain a description right after the type or no type at all.
    Returns (type_hint, initial_description_text).
    """
    s = initial_content.strip()
    if not s:
        return (None, "")
    # Expect type between the first pair of colons, e.g., ':int:' ...
    if s.startswith(":"):
        # find closing colon
        second_colon_idx = s.find(":", 1)
        if second_colon_idx != -1:
            type_hint = s[1:second_colon_idx].strip() or None
            remainder = s[second_colon_idx + 1 :].strip()
            # If remainder starts with a hyphen or 'returns', keep as description
            return (type_hint, remainder)
    # No type enclosed
    return (None, s)

# ----------------------------
# Main parser
# ----------------------------

class SpecialTagParser:
    def __init__(self, lines: List[str]):
        self.lines = lines
        self.docblocks: List[DocBlock] = []
        self.skip_ranges: List[Tuple[int, int]] = []  # inclusive line ranges to remove

    def parse(self) -> None:
        """
        Parses the lines to build docblocks and skip ranges for tag blocks.
        A tag block ends when the next tag starts or when a def/class line appears.
        """
        i = 0
        current_db: Optional[DocBlock] = None
        pending_tag: Optional[str] = None  # last tag type being collected (input/notes/example/output)

        while i < len(self.lines):
            line = self.lines[i]
            tag_match = TAG_PATTERN.match(line)
            def_match = DEF_PATTERN.match(line)
            class_match = CLASS_PATTERN.match(line)

            if tag_match:
                tag_name, trailing = tag_match.group(1), tag_match.group(2)
                if tag_name not in RECOGNIZED_TAGS:
                    # Unknown tag -> treat as comment
                    i += 1
                    continue

                if current_db is None:
                    current_db = DocBlock(start_line=i)
                    pending_tag = None

                # Set kind if this is an entity tag
                if tag_name in ("function", "class", "classFunction"):
                    # The trailing is the summary line
                    current_db.kind = tag_name
                    current_db.summary = CleanStringForPyiDescription(trailing.strip(),enforcePeriod=False)
                    pending_tag = tag_name
                elif tag_name == "input":
                    pending_tag = "input"
                    # trailing may be empty; content continues in subsequent lines until next tag/def/class
                elif tag_name == "output":
                    pending_tag = "output"
                    type_hint, desc = parse_output_header(trailing)
                    current_db.output = OutputSpec(type_hint=type_hint, 
                                                   description=CleanStringForPyiDescription(desc.strip()) or "")
                elif tag_name == "notes":
                    pending_tag = "notes"
                    # initial trailing content goes to notes if present
                    if trailing.strip():
                        current_db.notes.append(CleanStringForPyiDescription(trailing.strip()))
                elif tag_name == "example":
                    pending_tag = "example"
                    if trailing.strip():
                        current_db.examples.append(trailing.strip())
                elif tag_name == "author":
                    pending_tag = None
                    current_db.author = trailing.strip() or None
                elif tag_name == "date":
                    pending_tag = None
                    current_db.date = trailing.strip() or None
                elif tag_name == "belongsTo":
                    pending_tag = None
                    current_db.belongs_to = trailing.strip() or None
                elif tag_name == "status":
                    # Ignored but still part of the block
                    pending_tag = None

                i += 1
                continue

            # While collecting a block, collect content lines for specific tags
            if current_db is not None:
                if def_match or class_match:
                    # Finalize summary if needed (e.g., ensure it ends with a period)
                    if current_db.summary:
                        current_db.summary = CleanStringForPyiDescription(current_db.summary, enforcePeriod=True)
                    
                    # Tag block ends right before this definition line
                    current_db.end_line = i - 1 if i > 0 else 0
                    self.docblocks.append(current_db)
                    # Record skip range for the tag block lines
                    self.skip_ranges.append((current_db.start_line, current_db.end_line))
                    current_db = None
                    pending_tag = None
                    # Do not advance; we will handle the def/class normally
                    continue

                # Not a def/class; if it's a comment, feed it to pending tag
                if pending_tag in ("function", "class", "classFunction"):
                    cont = parse_notes_line(line) # Reuse notes parser to strip '#' and whitespace
                    if cont:
                        # Append with a space or newline to the existing summary
                        current_db.summary += " " + cont.strip() #avoid many spaces
                    i += 1
                    continue
                elif pending_tag == "input":
                    param = parse_input_line(line)
                    if param:
                        current_db.inputs.append(param)
                    i += 1
                    continue
                elif pending_tag == "notes":
                    note = parse_notes_line(line)
                    if note is not None:
                        current_db.notes.append(note)
                    i += 1
                    continue
                elif pending_tag == "example":
                    ex = parse_examples_line(line)
                    if ex is not None:
                        current_db.examples.append(ex)
                    i += 1
                    continue
                elif pending_tag == "output":
                    # Additional lines under output: treat as description continuation
                    cont = parse_notes_line(line)
                    if cont is not None and current_db.output is not None:
                        if current_db.output.description:
                            current_db.output.description += "\n" + cont
                        else:
                            current_db.output.description = cont
                    i += 1
                    continue
                else:
                    # We are inside a block but no active sub-tag; just keep moving
                    i += 1
                    continue

            # Normal non-tag line: just move on
            i += 1

        # If file ended while still collecting a docblock, finalize it
        if current_db is not None:
            current_db.end_line = len(self.lines) - 1
            self.docblocks.append(current_db)
            self.skip_ranges.append((current_db.start_line, current_db.end_line))

# ----------------------------
# Docstring renderer
# ----------------------------

from typing import Dict, Any, List

class GoogleDocstringRenderer:
    
    def needs_raw_string(self, data: Any) -> bool:
        """Recursively check if any string in the data contains a backslash."""
        if isinstance(data, str):
            return "\\" in data
        elif isinstance(data, list):
            return any(self.needs_raw_string(item) for item in data)
        elif isinstance(data, dict):
            return any(self.needs_raw_string(value) for value in data.values())
        return False

    def render(self, data: Dict[str, Any], indent: str) -> str:
        """Render a Google-style docstring from a dictionary.
        
        Expected keys:
          - kind: 'function' | 'class' | 'classFunction' (not used for formatting)
          - summary: str
          - description: str or list[str] (optional)
          - inputs: list of {name, description, type_hint?}
          - output: {type_hint?, description?} or None
          - notes: list[str]
          - examples: list[str]
          - author: str or None
          - date: str or None
          - belongs_to: str or None
        """
        lines: List[str] = []

        #determine if 'r' prefix needed:
        addR = self.needs_raw_string(data)
        quote_start = 'r"""' if addR else '"""'
        
        summary = (data.get("summary") or "").strip()
        if summary: #summary shall be in first line
            summary = summary[0].upper()+summary[1:] #otherwise docstring warning!
            if not summary.endswith('.'):
                summary+='.' #otherwise docstring warning!
            lines.append(f'{indent}{quote_start}{summary}')
        else:
            lines.append(f'{indent}{quote_start}')

        # extended description (free text under the summary)
        description = data.get("description")
        desc_lines: List[str] = []
        if isinstance(description, str) and description.strip() != '':
            desc_lines = [l.rstrip() for l in description.strip().splitlines()]
        elif isinstance(description, list):
            # join the list with newlines so callers can pass paragraphs in a list
            desc_lines = [l.rstrip() for l in "\n".join(description).splitlines()]

        if desc_lines:
            # blank line between summary and description
            lines.append(f"{indent}")
            for dl in desc_lines:
                lines.append(f"{indent}{dl}") 
            
        inputs = data.get("inputs") or []
        if inputs:
            if summary:
                lines.append(f"{indent}")
            lines.append(f"{indent}Args:")
            for p in inputs:
                name = p.get("name", "").strip()
                desc = (p.get("description") or "").strip()
                # Prefer provided type_hint; fall back to 'Any'
                type_part = (p.get("type_hint") or "Any")
                lines.append(f"{indent}    {name} ({type_part}): {desc}\n")

        output = data.get("output") or None
        if output and (output.get("type_hint") or output.get("description")):
            if summary and not inputs:
                lines.append(f"{indent}")
            lines.append(f"{indent}Returns:")
            t = output.get("type_hint") or "Any"
            desc = (output.get("description") or "").strip()
            if desc:
                first_line, *rest = desc.splitlines()
                lines.append(f"{indent}    {t}: {first_line}")
                for r in rest:
                    lines.append(f"{indent}        {r}")
            else:
                lines.append(f"{indent}    {t}")

        notes = [n for n in (data.get("notes") or []) if (n or "").strip() != ""]
        if notes:
            if summary and not inputs and not output:
                lines.append(f"{indent}")
            lines.append(f"{indent}Notes:")
            for nl in "\n".join(notes).splitlines():
                lines.append(f"{indent}    {nl.rstrip()}\n")

        examples = [e for e in (data.get("examples") or []) if (e or "").strip() != ""]
        if examples:
            lines.append(f"{indent}")
            lines.append(f"{indent}Examples:")
            for el in "\n".join(examples).splitlines():
                lines.append(f"{indent}    {el.rstrip()}")

        meta_lines: List[str] = []
        author = data.get("author") #author must not exist
        date = data.get("date")
        belongs_to = data.get("belongs_to")
        if author:
            meta_lines.append(f"Author: {author}")
        if date:
            meta_lines.append(f"Date: {date}")
        if belongs_to:
            meta_lines.append(f"Belongs to: {belongs_to}")
        if meta_lines:
            lines.append(f"{indent}")
            for ml in meta_lines:
                lines.append(f"{indent}{ml}")

        if len(lines) == 1:
            lines[0]+='"""' #should be only one line
        else:
            lines.append(f'{indent}"""') #multiline docstring
            
        return "\n".join(lines)
    
    def render_block(self, db: DocBlock, indent: str) -> str:
        """Convenience adapter to render a DocBlock."""
        return self.render(db.to_dict(), indent)
    
# ----------------------------
# Code transformer
# ----------------------------

class CodeTransformer:
    def __init__(self, lines: List[str], docblocks: List[DocBlock], skip_ranges: List[Tuple[int, int]]):
        self.lines = lines
        # Sort docblocks by start line to process in order
        self.docblocks = sorted(docblocks, key=lambda d: d.start_line or 0)
        # Merge skip ranges and sort
        self.skip_ranges = self._merge_ranges(sorted(skip_ranges))

    def _find_header_end(self, start_idx: int) -> int:
        """Given the index of a def/class start line, return the index of the line that closes the header (the one whose stripped text ends with ':').         Falls back to start_idx if not found."""
        j = start_idx
        while j < len(self.lines):
            # stop early if another tag starts (def/class headers must be contiguous)
            if TAG_PATTERN.match(self.lines[j]):
                break
            if self.lines[j].rstrip().endswith(":"):
                return j
            j += 1
        return start_idx
    
    @staticmethod
    def _merge_ranges(ranges: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Merge overlapping or contiguous ranges."""
        if not ranges:
            return []
        merged = []
        cur_s, cur_e = ranges[0]
        for s, e in ranges[1:]:
            if s <= cur_e + 1:
                cur_e = max(cur_e, e)
            else:
                merged.append((cur_s, cur_e))
                cur_s, cur_e = s, e
        merged.append((cur_s, cur_e))
        return merged

    @staticmethod
    def _line_is_def(line: str) -> bool:
        return DEF_PATTERN.match(line) is not None

    @staticmethod
    def _line_is_class(line: str) -> bool:
        return CLASS_PATTERN.match(line) is not None

    @staticmethod
    def _indent_of(line: str) -> str:
        return line[: len(line) - len(line.lstrip())]

    def transform(self) -> List[str]:
        renderer = GoogleDocstringRenderer()

        # Map from header-end line index to a tuple (DocBlock, header-start index)
        targets: Dict[int, Tuple[DocBlock, int]] = {}

        for db in self.docblocks:
            start_idx = None
            end_idx = None
            # Find the def/class start
            for j in range((db.end_line or -1) + 1, len(self.lines)):
                line = self.lines[j]
                if db.kind == "class" and self._line_is_class(line):
                    start_idx = j
                    break
                elif db.kind in ("function", "classFunction") and self._line_is_def(line):
                    start_idx = j
                    break
                if TAG_PATTERN.match(line):
                    break

            if start_idx is not None:
                # Find the header end (last line of the signature that ends with ':')
                end_idx = self._find_header_end(start_idx)
                targets[end_idx] = (db, start_idx)


        output_lines: List[str] = []
        skip_ptr = 0
        current_skip = self.skip_ranges[skip_ptr] if self.skip_ranges else None

        i = 0
        while i < len(self.lines):
            if current_skip and current_skip[0] <= i <= current_skip[1]:
                i = current_skip[1] + 1
                skip_ptr += 1
                current_skip = self.skip_ranges[skip_ptr] if skip_ptr < len(self.skip_ranges) else None
                continue

            output_lines.append(self.lines[i])

            if i in targets:
                db, start_idx = targets[i]
                # Use the indentation of the def/class line (block indentation)
                def_indent = self._indent_of(self.lines[start_idx])
                body_indent = def_indent + "    "
                docstr = renderer.render(db.to_dict(), indent=body_indent)
                output_lines.append(docstr)

            i += 1

        return output_lines
    
# ----------------------------
# CLI and orchestrator
# ----------------------------

def ConvertExuDoc2Docstrings(input_path: str, output_path: str) -> None:
    with open(input_path, "r", encoding="utf-8") as f:
        lines = f.read().splitlines()

    parser = SpecialTagParser(lines)
    parser.parse()

    transformer = CodeTransformer(lines, parser.docblocks, parser.skip_ranges)
    new_lines = transformer.transform()

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(new_lines) + ("\n" if new_lines and not new_lines[-1].endswith("\n") else ""))

# could be used externally:
def main():
    ap = argparse.ArgumentParser(description="Convert special #** tags to Google-style docstrings.")
    ap.add_argument("input", help="Path to input Python file")
    ap.add_argument("output", help="Path to output Python file")
    args = ap.parse_args()
    ConvertExuDoc2Docstrings(args.input, args.output)
    print('converted:',args.input, args.output)

if __name__ == "__main__":
    main()
    
    
    if False:
        #test for DocBlock and google renderer
        # Assuming the classes GoogleDocstringRenderer, DocBlock, Param, OutputSpec are imported
        
        data = {
        "kind": "function",
        "summary": "Create a mass point and its node.",
        "inputs": [
            {"name": "mbs", "description": "The MainSystem where items are created.", "type_hint": None},
            {"name": "name", "description": "Name string for the object."},
            {"name": "physicsMass"},
        ],
        #"output": {"type_hint": "Union[dict, ObjectIndex]", "description": "Object index or a dict if returnDict=True."},
        "output": {"description": "Object index or a dict if returnDict=True."},
        "notes": ["Set create2D=True to create 2D variants."],
        #"examples": ["MainSystemCreateMassPoint(mbs, physicsMass=1.0)"],
        #"author": "John Doe",
        #"date": "2024-01-01",
        #"belongs_to": "utilities.mass",
        }
        
        renderer = GoogleDocstringRenderer()
        docstring = renderer.render(data, indent=" "*4)
        print(docstring)


        