#measurement for docs/dev/design/symbolicIntermediateValues.md (revision2026c step RG12.2.5, #3048): the force and the
#position of userRigidBodyTest.py recorded with a small counting type instead of exudyn.symbolic - the number of
#operations of the graph (each shared node once) against the number a tree evaluation performs (a shared node at every
#use), which is what a flat evaluation in topological order would save
import contextlib
import io
import os

os.environ['EXUDYN_SUPPRESS_UI_WINDOW_OPEN'] = '1'


class Node:
    """a recorded operation and its operands; 'var' and 'const' are leaves"""
    def __init__(self, op, args=()):
        self.op, self.args = op, args
        self._tree = None

    def Tree(self):
        """the operations a tree evaluation performs for this node"""
        if self._tree is None:
            self._tree = (0 if self.op in ('var', 'const') else 1) + sum(a.Tree() for a in self.args)
        return self._tree


def Wrap(x):
    return x if isinstance(x, Node) else Node('const')


for (name, op) in [('__add__', '+'), ('__sub__', '-'), ('__mul__', '*'), ('__truediv__', '/')]:
    setattr(Node, name, (lambda o: lambda a, b: Node(o, (a, Wrap(b))))(op))
for (name, op) in [('__radd__', '+'), ('__rsub__', '-'), ('__rmul__', '*')]:
    setattr(Node, name, (lambda o: lambda a, b: Node(o, (Wrap(b), a)))(op))
Node.__neg__ = lambda a: Node('neg', (a,))


class Functions:
    """the functions the component functions of the test model call"""
    @staticmethod
    def sin(a):
        return Node('sin', (Wrap(a),))

    @staticmethod
    def cos(a):
        return Node('cos', (Wrap(a),))


def GraphOperations(roots):
    """the operations of the graph, each shared node once"""
    seen = set()

    def Visit(node):
        if id(node) in seen or node.op in ('var', 'const'):
            return
        seen.add(id(node))
        for a in node.args:
            Visit(a)
    for r in roots:
        Visit(r)
    return len(seen)


fileName = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'python', 'TestModels', 'userRigidBodyTest.py')
source = open(fileName, encoding='utf-8').read()
source = source[:source.index('#the matrix results as symbolic matrices')] #the definitions of the component functions
definitions = {}
with contextlib.redirect_stdout(io.StringIO()):
    exec(source, definitions)
q = [Node('var') for i in range(6)]
q_t = [Node('var') for i in range(6)]
for (name, roots) in [('force', definitions['ForceComponents'](Functions, q, q_t)),
                      ('position', definitions['PositionComponents'](Functions, [0.2, 0.1, 0.05], q))]:
    roots = [r for r in roots if isinstance(r, Node)]
    tree = sum(r.Tree() for r in roots)
    graph = GraphOperations(roots)
    print(name.ljust(10), 'tree evaluation: %4d operations, graph: %4d operations, ratio %.1f' % (tree, graph, tree/graph))
