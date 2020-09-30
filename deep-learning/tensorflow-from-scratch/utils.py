import number_tree_flow


def traverse_tree(root, level=0):
    import json

    def helper(root):
        data = {}
        data[root.id] = [helper(c) for c in root.children]
        return data
    data = helper(root)
    print(json.dumps(data,sort_keys=True, indent=2))
