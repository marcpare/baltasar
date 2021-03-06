# ---------------------------------------------------------
#
# Implement Realpaver constructs as Python objects.
#
# quick_constraint implemented using custom grammar
#
# ---------------------------------------------------------

class Constant:
  def __init__(self, var_name, value):
    self.var_name = var_name
    self.value = value
  
  def render(self):
    return "%s = %s" % (self.var_name, self.value)
    
class Variable:
  def __init__(self, var_name, lower_bound, upper_bound, defined_over=None, var_type=None):
    self.var_name = var_name
    self.lower_bound = lower_bound
    self.upper_bound = upper_bound
    self.defined_over = defined_over
    
    if var_type:
      self.var_name = "%s %s" % (var_type, var_name)
  
  def render(self):
    left_bracket = "["
    if self.lower_bound == "-oo":
      left_bracket = "]"
    right_bracket = "]"
    if self.upper_bound == "+oo":
      right_bracket = "["
    
    if self.defined_over:
      a = self.defined_over[0]
      b = self.defined_over[-1]
      return "%s[%s..%s] in %s%s, %s%s" % (self.var_name, a, b, left_bracket, self.lower_bound, self.upper_bound, right_bracket)
    else:
      return "%s in %s%s, %s%s" % (self.var_name, left_bracket, self.lower_bound, self.upper_bound, right_bracket)
      
constants = []
variables = []
variable_domain = {}
constant_domain = {}
constraints = []
  
def add_constant(cname, cvalue, defined_over=None):
  constant_domain[cname] = defined_over
  # unroll constants defined over an interval
  if defined_over:
    for (x, y) in zip(defined_over, cvalue):
      constants.append(Constant("%s_%s" % (cname, x), y))
  else:
    constants.append(Constant(cname, cvalue))
  
def add_variable(var_name, lower_bound, upper_bound, defined_over=None, var_type=None):
  variable_domain[var_name] = defined_over
  variables.append(Variable(var_name, lower_bound, upper_bound, defined_over, var_type))

def add_constraint(constraint):
  try:
    constraints.extend(constraint)
  except:
    constraints.append(constraint())
  
def render():
  ret = "Constants\n"
  rendered_constants = " , \n".join(["  " + c.render() for c in constants])
  ret += rendered_constants + " ;"
  
  ret += "\n\n"
  ret += "Variables\n"
  rendered_variables = " , \n".join(["  " + v.render() for v in variables])
  ret += rendered_variables + " ;"
  
  ret += "\n\n"
  ret += "Constraints\n"
  rendered_constraints = " , \n".join(["  " + c for c in constraints])
  ret += rendered_constraints + " ;"
  
  return ret

def define_over(defined_over, constraint):
  return [constraint(x) for x in defined_over]
  
def make_callable(constraint):
  def var_render(x=None):
    if constraint in constant_domain and constant_domain[constraint] is not None:
      if x is None:
        print "Missing domain for constant %s" % constraint
      return "%s_%s" % (constraint, x)
    elif constraint in variable_domain and variable_domain[constraint] is not None:
      return "%s[%s]" % (constraint, x)        
    else:
      return constraint      
  def constant_render(x=None):
    return constraint
  def constraint_render(x=None):
    return constraint(x)
  
  if isinstance(constraint, str):
    return var_render
  elif isinstance(constraint, int) or isinstance(constraint, float):
    return constant_render
  else:
    return constraint_render

def create_operator(template_str):
  def operater_func(*args):
    args = map(make_callable, args)
    def render_operator(x=None):
      evaluated = [arg(x) for arg in args]
      return template_str % tuple(evaluated)
    return render_operator
  return operater_func

eq = create_operator("%s = %s")
lt = create_operator("%s <= %s")
gt = create_operator("%s >= %s")
prod = create_operator("(%s * %s)")
div = create_operator("(%s / %s)")
add = create_operator("(%s + %s)")
sub = create_operator("(%s - %s)")
neg = create_operator("(-%s)")
pos = create_operator("(+%s)")
power = create_operator("(%s^%s)")
max = create_operator("max(%s, %s)")

def lag(var_name, x=None):
  def inner_lag(x=None):
    return "%s[%s]" % (var_name, x-1)
  return inner_lag
def lead(var_name, x=None):
  def inner_lead(x=None):
    return "%s[%s]" % (var_name, x+1)
  return inner_lead

def var_sum(domain, constraint):
  return " + ".join([constraint(x) for x in domain])
  
def make_parser():
    import ply.lex as lex
    import ply.yacc as yacc

    # ---------------------------------
    # Tokenizing rules
    # ---------------------------------

    reserved = {
       'for' : 'FOR',
       'in' : 'IN'
    }

    tokens = [
        'ID','FLOAT','INT',
        'PLUS','MINUS', 'TIMES','DIVIDE','EQUALS','UMINUS','UPLUS',
        'GREATER_THAN', 'LESS_THAN',
        'LPAREN','RPAREN','LBRACK','RBRACK',
        ]  + list(reserved.values())

    # Tokens
    t_PLUS    = r'\+'
    t_MINUS   = r'-'
    t_TIMES   = r'\*'
    t_DIVIDE  = r'/'
    t_EQUALS  = r'='
    t_LESS_THAN  = r'<'
    t_GREATER_THAN  = r'>'
    t_LPAREN  = r'\('
    t_RPAREN  = r'\)'
    t_LBRACK  = r'\['
    t_RBRACK  = r'\]'

    def t_ID(t):
        r'[a-zA-Z_][a-zA-Z_0-9]*'
        t.type = reserved.get(t.value,'ID')    # Check for reserved words
        return t

    t_ignore = " \t"

    def t_FLOAT(t):
        r'[-+]?[0-9]*\.[0-9]+([eE][-+]?[0-9]+)?'
        t.value = float(t.value)                
        return t

    def t_INT(t):
        r'\d+'
        t.value = int(t.value)
        return t

    def t_newline(t):
        r'\n+'
        t.lexer.lineno += t.value.count("\n")

    def t_error(t):
        print("Illegal character '%s'" % t.value[0])
        t.lexer.skip(1)

    # Build the lexer
    lexer = lex.lex()

    # ---------------------------------
    # Calculator parsing rules
    # ---------------------------------

    precedence = (
        ('left','PLUS','MINUS'),
        ('left','TIMES','DIVIDE'),
        ('right','UMINUS','UPLUS'),
        ('left', 'EQUALS'),
        ('left', 'FOR'),
        )

    def p_statement_eqn(p):
        'statement : equation'
        p[0] = p[1]

    def p_expression_equation(p):
        '''equation : expression EQUALS expression
                    | expression LESS_THAN expression
                    | expression GREATER_THAN expression '''
        p[0] = ('equation-expression', p[2], p[1], p[3])

    def p_statement_for(p):
        '''statement : equation FOR ID IN ID '''
        p[0] = ('for-expression', p[3], p[5], p[1])

    def p_expression_binop(p):
        '''expression : expression PLUS expression
                      | expression MINUS expression
                      | expression TIMES expression
                      | expression DIVIDE expression'''
        # if p[2] == '+'  : p[0] = p[1] + p[3]
        p[0] = ('binary-expression',p[2],p[1],p[3])

    def p_expression_group(p):
        '''expression : LPAREN expression RPAREN '''
        p[0] = ('group-expression',p[2])

    def p_expression_int(p):
        'expression : INT'
        p[0] = ('int-expression',p[1])

    def p_expression_float(p):
        'expression : FLOAT'
        p[0] = ('float-expression',p[1])

    def p_expression_uminus(p):
        '''expression : MINUS expression %prec UMINUS
                      | PLUS expression %prec UPLUS '''
        p[0] = ('unary-expression',p[2],p[1])

    def p_expression_array(p):
        "expression : ID LBRACK expression RBRACK"
        p[0] = ('array-expression',p[1], p[3])

    def p_expression_id(p):
        "expression : ID"
        p[0] = ('variable-expression',p[1])

    def p_error(p):
        if p:
            print("Syntax error at '%s'" % p.value)
        else:
            print("Syntax error at EOF")


    # Build the parser
    parser = yacc.yacc()

    # ---------------------------------
    # Input function 
    # ---------------------------------

    def input(text):
        result = parser.parse(text,lexer=lexer)
        return result

    return input
    
ast = make_parser()
var_lookup = {}
sets = {}

def parse(node):
  # determine the type of node
  if node[0] == 'binary-expression':
    if node[1] == '+':
      return add(parse(node[2]), parse(node[3]))
    elif node[1] == '*':
      return prod(parse(node[2]), parse(node[3]))
    elif node[1] == '/':
      return div(parse(node[2]), parse(node[3]))
    elif node[1] == '-':
      return sub(parse(node[2]), parse(node[3]))
  elif node[0] == 'unary-expression':
    if node[2] == '-':
      return neg(parse(node[1]))
    elif node[2] == '+':
      return pos(parse(node[1]))
  elif node[0] == 'float-expression' or node[0] == 'int-expression':
    return node[1]
  elif node[0] == 'variable-expression':
    if node[1] not in constant_domain and node[1] not in variable_domain and node[1] not in var_lookup:
      print "Did not add %s to the problem" % node[1]
    else:
      return var_lookup.get(node[1], node[1])
  elif node[0] == 'array-expression':
    index = parse(node[2])
    try:
      index = eval(index())
    except:
      pass        
    if node[1] in constant_domain and constant_domain[node[1]] is not None:
      return "%s_%s" % (node[1], index)
    elif node[1] in variable_domain and variable_domain[node[1]] is not None:
      return "%s[%s]" % (node[1], index)
    elif node[1] in constant_domain and constant_domain[node[1]] is None:
      print "Constant %s referenced as a variable" % node[1]
    elif node[1] in variable_domain and variable_domain[node[1]] is None:
      print "Variable %s referenced as a constant" % node[1]
    else:
      print "Array expression neither a constant nor a variable."
    
  elif node[0] == 'for-expression':
    over_set = sets[node[2]]
    ret = []
    for x in over_set:
      var_lookup[node[1]] = x
      ret.append(parse(node[3]))
    return ret
  elif node[0] == 'equation-expression':
    if node[1] == '=':
      return eq(parse(node[2]), parse(node[3]))
    elif node[1] == '<':
      return lt(parse(node[2]), parse(node[3]))
    elif node[1] == '>':
      return gt(parse(node[2]), parse(node[3]))
  elif node[0] == 'group-expression':
    return parse(node[1])

def parse_constraint(raw_equation):
  foo = ast(raw_equation)
  print foo
  return parse(foo)

def quick_constraint(raw_equation):
  parsed = parse_constraint(raw_equation)
  
  if isinstance(parsed, list):
    for x in parsed:
      unfolded = x()
      add_constraint(x)
  else:
    add_constraint(parsed)    

def add_set(set_name, set_items):
  sets[set_name] = set_items  

if __name__ == "__main__":

  sets = {'after_first': range(1, 5)}
  var_lookup = {'x': [1, 2, 3, 4, 5],
                'accumPrecharge': 75000.0}

  print parse_constraint("accumPressure[0] = accumPrecharge")()
  cs =  parse_constraint("accumPressure[t-1]  = 2*accumPressure[t] + 1.1 for t in after_first")
  print cs
  for x in cs:
    print x()
  
  print parse_constraint("V = -P*D")()