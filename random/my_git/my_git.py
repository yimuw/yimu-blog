import os
import hashlib
import pickle
import argparse
import sys

join = os.path.join

class TreeNode:
    def __init__(self, file_path, parents=[]):
        self.file_path = file_path
        self.parents = parents
        self.children = []
        self.file_content = None
        self.hash_val = None

    def compute_hash(self):
        self.file_content = open(self.file_path, 'r').read()
        self.hash_val = hashlib.sha1(self.file_content.encode('utf-8')).hexdigest()

class MyGit:
    class States:
        def __init__(self):
            self.head = None
            self.tracked_files = set()


    def __init__(self, dir = '.'):
        self.git_dir = 'my_git'
        self.objects_dir = join(dir, self.git_dir, 'objects')
        self.states_path = join(dir, self.git_dir, 'states')
        self.states = self.States()

    def init(self):
        if os.path.exists(self.git_dir):
            print('git dir exist!')
            return
        os.mkdir(self.git_dir)
        os.mkdir(self.objects_dir)

        
        root = TreeNode('ROOT')
        root.hash_val = 'ROOT'
        root.file_content = ''

        self.states.head = 'ROOT'

        self.save_node(root)
        self.save_states()

        print('init my_git repo!')

    def load_states(self):
        self.states = pickle.load( open( join(self.git_dir, 'states'), "rb" ) )

    def save_states(self):
        pickle.dump( self.states, open( join(self.git_dir, 'states'), "wb" ) )

    def load_node(self, hash):
        return pickle.load(open( join(self.objects_dir, hash), "rb" ) )

    def save_node(self, node):
        pickle.dump(node, open( join(self.objects_dir, node.hash_val), "wb" ) )

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
            
        new_node = TreeNode(file_path, parents = [self.states.head])
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

    def checkout(self, id):
        objects = os.listdir(self.objects_dir)
        if id == 'ROOT':
            print('can not checkout virtual node root')
            return 

        if id in objects:
            print('checkout hash id...')
            print('loading object from:', join(self.objects_dir, id))
            node = pickle.load(open( join(self.objects_dir, id), "rb" ) )

            with open(node.file_path, 'w') as file:
                file.write(node.file_content)
        
            self.states.head = node.hash_val

            print('checkout done!')

    def log(self):
        def tree_recursion(node_hash):
            if node_hash is None:
                return

            node = self.load_node(node_hash)

            print('======================')
            print('node hash: ', node.hash_val)
            print('file path:', node.file_path)
            print('file content:', node.file_content)
            print('children: ', node.children)
            print('parents: ', node.parents)
            print('======================')

            for p in node.parents:
                tree_recursion(p)

        self.load_states()
        tree_recursion(self.states.head)


if __name__ == "__main__":
    git = MyGit()

    argparser = argparse.ArgumentParser()
    argsubparsers = argparser.add_subparsers(title="Commands", dest="command")
    argsubparsers.required = True

    argsubparsers.add_parser("init")
    argsubparsers.add_parser("status")
    argsubparsers.add_parser("log")

    argsp = argsubparsers.add_parser("commit")
    argsp.add_argument('file_path',
                    help='file to commit')

    argsp = argsubparsers.add_parser("checkout")
    argsp.add_argument('id',
                    help='id is a hash or a branch')

    args = argparser.parse_args(sys.argv[1:])

    if args.command == "status":
        git.status()
    elif args.command == "checkout":
        git.checkout(args.id)
    elif args.command == "commit":
        git.commit(args.file_path)
    elif args.command == "init":
        git.init()
    elif args.command == "log":
        git.log()


