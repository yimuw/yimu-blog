import variables_tree_flow


def traverse_tree(root, level=0):
    import json

    def helper(root):
        data = {}
        if isinstance(root, variables_tree_flow.Variable):
            data[root.id] = {'val': root.value, 'grad': root.grad,
                             'children': [helper(c) for c in root.children]}
        else:
            data[root.id] = {'children': [helper(c) for c in root.children]}
        return data
    data = helper(root)
    print(json.dumps(data, indent=2))
