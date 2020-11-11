import os
import hashlib
import pickle
import argparse
import sys

join = os.path.join

class TreeNode:
    def __init__(self, file_path, parent=None):
        self.file_path = file_path
        self.file_content = open(self.file_path, 'r').read()
        self.hash_val = hashlib.sha1(self.file_content.encode('utf-8')).hexdigest()
        self.parent = parent


class MyGit:
    def __init__(self, dir = '.'):
        self.git_dir = 'my_git'
        self.objects_dir = join(dir, self.git_dir, 'objects')
        self.states_path = join(dir, self.git_dir, 'states')
        self.states = {
            'head' : None
        }

    def init(self):
        if os.path.exists(self.git_dir):
            print('git dir exist!')
            return
        os.mkdir(self.git_dir)
        os.mkdir(self.objects_dir)

        self.save_states()

    def load_states(self):
        self.states = pickle.load( open( join(self.git_dir, 'states'), "rb" ) )

    def save_states(self):
        pickle.dump( self.states, open( join(self.git_dir, 'states'), "wb" ) )

    def status(self):
        self.load_states()
        print('HEAD: ', self.states['head'])

    def commit(self, file_path):
        self.load_states()
        print('current HEAD:', self.states['head'])

        if not os.path.exists(file_path):
            print('file does not exist')
            return
            
        node = TreeNode(file_path, parent = self.states['head'])

        if node.hash_val == self.states['head']:
            print('nothing to commit!')
            return

        self.states['head'] = node.hash_val
        self.save_states()
        # save the object
        pickle.dump( node, open( join(self.objects_dir, node.hash_val), "wb" ) )

        print('commit success! ')
        print('new HEAD:', self.states['head'])

    def checkout(self, id):
        objects = os.listdir(self.objects_dir)
        if id in objects:
            print('checkout hash id...')
            print('loading object from:', join(self.objects_dir, id))
            node = pickle.load(open( join(self.objects_dir, id), "rb" ) )

            with open(node.file_path, 'w') as file:
                file.write(node.file_content)
        
            self.states['head'] = node.hash_val

            print('checkout done!')
        


if __name__ == "__main__":
    git = MyGit()

    argparser = argparse.ArgumentParser()
    argsubparsers = argparser.add_subparsers(title="Commands", dest="command")
    argsubparsers.required = True

    argsp = argsubparsers.add_parser("init")

    argsp = argsubparsers.add_parser("status")

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
        pass


