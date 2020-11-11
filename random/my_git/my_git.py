import os
import hashlib
import pickle
import argparse
import sys

join = os.path.join


class TreeNode:
    def __init__(self, file_path, new_file, parents=[]):
        self.file_path = file_path
        self.new_file = new_file
        self.parents = parents
        self.children = []
        self.file_content = None
        self.hash_val = None

    def compute_hash(self):
        self.file_content = open(self.file_path, 'r').read()
        self.hash_val = hashlib.sha1(
            self.file_content.encode('utf-8')).hexdigest()


class MyGit:
    class States:
        def __init__(self):
            self.head = None
            self.tracked_files = set()

    def __init__(self, dir='.'):
        self.git_dir = 'my_git'
        self.objects_dir = join(dir, self.git_dir, 'objects')
        self.states_path = join(dir, self.git_dir, 'states')
        self.states = self.States()
        self.root_name = 'ROOT'

    def init(self):
        if os.path.exists(self.git_dir):
            print('git dir exist!')
            return
        os.mkdir(self.git_dir)
        os.mkdir(self.objects_dir)

        root = TreeNode(self.root_name, True)
        root.hash_val = self.root_name
        root.file_content = ''

        self.states.head = self.root_name

        self.save_node(root)
        self.save_states()

        print('init my_git repo!')

    def load_states(self):
        self.states = pickle.load(open(join(self.git_dir, 'states'), 'rb'))

    def save_states(self):
        pickle.dump(self.states, open(join(self.git_dir, 'states'), 'wb'))

    def load_node(self, hash):
        return pickle.load(open(join(self.objects_dir, hash), 'rb'))

    def save_node(self, node):
        pickle.dump(node, open(join(self.objects_dir, node.hash_val), 'wb'))

    def status(self):
        self.load_states()
        print('HEAD: ', self.states.head)
        print('tracked files:', self.states.tracked_files)

    def commit(self, file_path):
        self.load_states()
        print('current HEAD:', self.states.head)

        if not os.path.exists(file_path):
            print('file does not exist')
            return

        new_node = TreeNode(
            file_path, new_file=file_path not in self.states.tracked_files, parents=[self.states.head])
        new_node.compute_hash()

        # using hash to detect changes
        if new_node.hash_val == self.states.head:
            print('nothing to commit!')
            return

        # update the children of current head node
        head_node = self.load_node(self.states.head)
        head_node.children.append(new_node.hash_val)
        self.save_node(head_node)

        # head point to new node
        self.states.head = new_node.hash_val
        self.states.tracked_files.add(file_path)
        self.save_states()

        # save the new node
        self.save_node(new_node)

        print('commit success! ')
        print('new HEAD:', self.states.head)

    def __find_path_to_node(self, cur_node_hash, target_node_hash, path):
        cur_node = self.load_node(cur_node_hash)
        path.append(cur_node.hash_val)

        if cur_node.hash_val == target_node_hash:
            return True

        for c in cur_node.children:
            if self.__find_path_to_node(c, target_node_hash, path):
                return True

        path.pop()
        print(path)
        return False

    def __lowest_common_ancester_path(self, node_hash1, node_hash2):
        print('finding LCA path between', node_hash1, node_hash2)
        path1, path2 = [], []

        if self.__find_path_to_node(self.root_name, node_hash1, path1) \
           and self.__find_path_to_node(self.root_name, node_hash2, path2):
            idx = 0
            for i in range(min(len(path1), len(path2))):
                if path1[i] != path2[i]:
                    break
                idx = i
            return path1[idx:][::-1] + path2[idx+1:]

        else:
            print('fail to find a LCA path')
            return []

    def checkout(self, id):
        objects = os.listdir(self.objects_dir)
        if id == self.root_name:
            print('can not checkout virtual node root')
            return

        if id in objects:
            print('checkout hash id...')
            self.load_states()
            path = self.__lowest_common_ancester_path(self.states.head, id)
            print('LCA path:', path)

            for i in range(len(path) - 1):
                cur_node_hash = path[i]
                next_node_hash = path[i+1]

                print('go to hash:', next_node_hash)
                self.states.head = next_node_hash

                cur_node = self.load_node(cur_node_hash)
                if cur_node.new_file:
                    os.remove(cur_node.file_path)
                    self.states.tracked_files.remove(cur_node.file_path)

                next_node = self.load_node(next_node_hash)
                self.states.tracked_files.add(next_node.file_path)
                with open(next_node.file_path, 'w') as file:
                    file.write(next_node.file_content)

            assert(self.states.head == id)
            self.save_states()

    def log(self):
        def tree_recursion(node_hash):
            if node_hash is None:
                return

            node = self.load_node(node_hash)

            print('======================')
            print('node hash: ', node.hash_val)
            print('file path:', node.file_path)
            print('new file:', node.new_file)
            print('file content:', node.file_content)
            print('children: ', node.children)
            print('parents: ', node.parents)
            print('======================')

            for p in node.parents:
                tree_recursion(p)

        self.load_states()
        tree_recursion(self.states.head)

    def gitk(self):
        import networkx as nx
        from matplotlib import pyplot as plt

        self.load_states()
        
        def get_edges(node_hash):
            node = self.load_node(node_hash)

            edges = [(node.hash_val, self.load_node(c).hash_val)
                     for c in node.children]
            for c in node.children:
                edges += get_edges(c)
            return edges

        graph = nx.DiGraph()

        edges = get_edges(self.root_name)
        edges.append(('HEAD', self.states.head))

        graph.add_edges_from(edges)
        plt.tight_layout()
        nx.draw_networkx(graph, arrows=True)
        plt.show()


if __name__ == '__main__':
    git = MyGit()

    argparser = argparse.ArgumentParser()
    argsubparsers = argparser.add_subparsers(title='Commands', dest='command')
    argsubparsers.required = True

    argsubparsers.add_parser('init')
    argsubparsers.add_parser('status')
    argsubparsers.add_parser('log')
    argsubparsers.add_parser('test')
    argsubparsers.add_parser('gitk')

    argsp = argsubparsers.add_parser('commit')
    argsp.add_argument('file_path',
                       help='file to commit')

    argsp = argsubparsers.add_parser('checkout')
    argsp.add_argument('id',
                       help='id is a hash or a branch')

    args = argparser.parse_args(sys.argv[1:])

    if args.command == 'status':
        git.status()
    elif args.command == 'checkout':
        git.checkout(args.id)
    elif args.command == 'commit':
        git.commit(args.file_path)
    elif args.command == 'init':
        git.init()
    elif args.command == 'log':
        git.log()
    elif args.command == 'gitk':
        git.gitk()
