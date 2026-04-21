import ast

NODES = (
    ast.Expression,
    ast.Lambda,
    ast.arguments,
    ast.arg,
    ast.Load,
    ast.Name,
    ast.Constant,
    ast.Attribute,
    ast.Subscript,
    ast.BoolOp,
    ast.And,
    ast.Or,
    ast.UnaryOp,
    ast.Not,
    ast.Is,
    ast.IsNot,
    ast.In,
    ast.NotIn,
    ast.BinOp,
    ast.Add,
    ast.Sub,
    ast.Mult,
    ast.Div,
    ast.Mod,
    ast.USub,
    ast.Compare,
    ast.Eq,
    ast.NotEq,
    ast.Lt,
    ast.LtE,
    ast.Gt,
    ast.GtE,
)

FUNCTIONS = {
    "abs": abs,
    "min": min,
    "max": max,
    "len": len,
    "hash": hash,
    "str": str,
}

def _validate(tree, arity):
    if not isinstance(tree.body, ast.Lambda):
        return False

    if len(tree.body.args.args) != arity:
        return False

    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            if not isinstance(node.func, ast.Name):
                return False
            if node.func.id not in FUNCTIONS:
                return False
        elif not isinstance(node, NODES):
            return False

    return True

def compile_lambda(lambda_str, arity):
    tree = ast.parse(lambda_str, mode="eval")
    if not _validate(tree, arity):
        return None

    safe_globals = {"__builtins__": {}}
    safe_globals.update(FUNCTIONS)

    return eval(
        compile(tree, "<eval_func>", "eval"),
        safe_globals,
        {},
    )

def aggregate_bool(bools, mode):
    if not bools:
        return False

    if mode == "any":
        return any(bools)
    elif mode == "all":
        return all(bools)
    else:
        return False

def eval_dict(variants_dict, eval_lambda, aggregation):
    results = []
    for _, value in variants_dict.items():
        results.append(bool(eval_lambda(value)))
    return aggregate_bool(results, aggregation)
