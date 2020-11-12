import os
import hashlib
import pickle
import argparse
import sys
import shutil
import copy

join = os.path.join


class TreeNode:
    def __init__(self, id, blobs=[], parents=[]):
        self.id = id
        self.blobs = blobs
        self.parents = parents
        self.children = []


class Blob:
    def __init__(self, file_path):
        self.file_path = file_path
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
        self.blobs_dir = join(dir, self.git_dir, 'blobs')
        self.nodes_dir = join(dir, self.git_dir, 'nodes')
        self.states_path = join(dir, self.git_dir, 'states')
        self.states = self.States()
        self.root_name = 'ROOT'

    def init(self):
        if os.path.exists(self.git_dir):
            print('git dir exist!')
            return
        os.mkdir(self.git_dir)
        os.mkdir(self.blobs_dir)
        os.mkdir(self.nodes_dir)

        root = TreeNode(self.root_name)

        self.states.head = root.id

        self.save_node(root)
        self.save_states()

        print('init my_git repo!')

    def load_states(self):
        self.states = pickle.load(open(join(self.git_dir, 'states'), 'rb'))

    def save_states(self):
        pickle.dump(self.states, open(join(self.git_dir, 'states'), 'wb'))

    def load_node(self, node_id):
        return pickle.load(open(join(self.nodes_dir, node_id), 'rb'))

    def save_node(self, node):
        pickle.dump(node, open(join(self.nodes_dir, node.id), 'wb'))

    def load_blob(self, blob_hash):
        return pickle.load(open(join(self.blobs_dir, blob_hash), 'rb'))

    def save_blob(self, blob):
        pickle.dump(blob, open(join(self.blobs_dir, blob.hash_val), 'wb'))

    def status(self):
        self.load_states()
        print('HEAD: ', self.states.head)
        print('tracked files:', self.states.tracked_files)
        for f in self.states.tracked_files:
            print('content of f:', f)
            print(open(f, 'r').read())

    def add(self, files):
        self.load_states()
        for f in files:
            if os.path.exists(f):
                self.states.tracked_files.add(f)
            else:
                print(f, 'does not exist!')
        self.save_states()

    def commit(self, message, verbose=True):
        self.load_states()
        print('current HEAD:', self.states.head)

        has_change = False

        all_blob = []
        for f in self.states.tracked_files:
            blob = Blob(f)
            all_blob.append(blob.hash_val)
            if blob.hash_val not in os.listdir(self.blobs_dir):
                has_change = True
                self.save_blob(blob)

        # using hash to detect changes
        if has_change == False:
            print('nothing to commit!')
            return

        new_node = TreeNode(message, blobs=all_blob,
                            parents=[self.states.head])

        # update the children of current head node
        head_node = self.load_node(self.states.head)
        head_node.children.append(new_node.id)
        self.save_node(head_node)

        # head point to new node
        self.states.head = new_node.id
        self.save_states()

        # save the new node
        self.save_node(new_node)
        if verbose:
            print('commit success! ')
            print('new HEAD:', self.states.head)

    def __merge_commit(self, p1, p2):
        merge_node = TreeNode(
            'merge-{}-{}'.format(p1[:6], p2[:6]), parents=[p1, p2])
        merge_node.hash_val = 'merge-{}-{}'.format(p1[:6], p2[:6])

        # update the children of p1, p2 node
        p1_node = self.load_node(p1)
        p1_node.children.append(merge_node.hash_val)
        self.save_node(p1_node)

        p2_node = self.load_node(p2)
        p2_node.children.append(merge_node.hash_val)
        self.save_node(p2_node)

        # head point to new node
        self.states.head = merge_node.hash_val
        self.save_states()

        # save the new node
        self.save_node(merge_node)

        print('merged commit success! ')
        print('new HEAD:', self.states.head)

    def checkout(self, node_id, verbose=True):
        if node_id not in os.listdir(self.nodes_dir):
            print('commit doesn not exist')
            return

        self.load_states()

        for f in self.states.tracked_files:
            os.remove(f)
        self.states.tracked_files = set()

        node = self.load_node(node_id)
        for blob_hash in node.blobs:
            blob = self.load_blob(blob_hash)
            self.states.tracked_files.add(blob.file_path)
            with open(blob.file_path, 'w') as file:
                file.write(blob.file_content)
        self.states.head = node_id
        self.save_states()

        if verbose:
            print('checkout success!')
            print('new HEAD:', self.states.head)

    def __lowest_common_ancester_path(self, node1_id, node2_id):
        print('finding LCA path between', node1_id, node2_id)

        self.lca = None

        def recursion(node_id):
            node = self.load_node(node_id)

            value = 0
            for c in node.children:
                value += recursion(c)

            value += 1 if node.id == node1_id or node.id == node2_id else 0

            if self.lca is None and value == 2:
                self.lca = node.id
            return value
        recursion(self.root_name)

        return self.lca

    def __detect_diff3_conflict(self, path):
        return '<<<<<<<' in open(path, 'r').read()

    def merge(self, node_id):
        if node_id not in os.listdir(self.nodes_dir):
            print('commit doesn not exist')
            return

        self.load_states()
        lca = self.__lowest_common_ancester_path(self.states.head, node_id)
        print('LCA node:', lca)

        if lca == node_id or lca == self.states.head:
            print('fastforward...')
            self.checkout(node_id)
            return

        merge_id = 'merge-{}-{}'.format(self.states.head, node_id)
        merge_dir = join(self.git_dir, merge_id)
        source_dir = join(merge_dir, 'source')
        target_dir = join(merge_dir, 'target')
        lca_dir = join(merge_dir, 'lca')

        if os.path.exists(merge_dir):
            shutil.rmtree(merge_dir)
        os.mkdir(merge_dir)
        os.mkdir(source_dir)
        os.mkdir(target_dir)
        os.mkdir(lca_dir)

        def snapshot(dir):
            for f in self.states.tracked_files:
                shutil.copyfile(f, join(dir, f))

        cur_head = self.states.head

        snapshot(source_dir)
        source_files = copy.deepcopy(self.states.tracked_files)

        self.checkout(lca, verbose=False)
        snapshot(lca_dir)
        lca_files = copy.deepcopy(self.states.tracked_files)

        self.checkout(node_id, verbose=False)
        snapshot(target_dir)
        target_files = copy.deepcopy(self.states.tracked_files)

        self.checkout(cur_head, verbose=False)

        all_files = target_files.union(source_files).union(lca_files)

        for f in all_files:
            print('...merging...', f)
            sf = join(source_dir, f)
            tf = join(target_dir, f)
            lf = join(lca_dir, f)

            # 1. f in lca, source and target.
            # do a diff3
            if f in source_files and f in target_files and f in lca_files:
                os.system('diff3 {} {} {} -m > {}'.format(sf, lf, tf, f))
                self.states.tracked_files.add(f)
                print('3 way merge')
                if self.__detect_diff3_conflict(f):
                    print(
                        '!!!! Merge conflict for {}!!!! please address in another commit!'.format(f))
            # 2. f in source and lca, but not target.
            # take source
            elif f in source_files and f in lca_files:
                shutil.copyfile(sf, f)
                self.states.tracked_files.add(f)
                print('new change from source. take from source')
            # 3. f in target and lca, but not source.
            # take target
            elif f in target_files and f in lca_files:
                shutil.copyfile(tf, f)
                self.states.tracked_files.add(f)
                print('new change from target. take from target')
            # 4. f in source or (in source and target).
            # new file. take source
            elif f in source_files:
                shutil.copyfile(sf, f)
                self.states.tracked_files.add(f)
                print('new file. take from source')
            # 5. f only in target.
            # new file. take target
            elif f in target_files:
                shutil.copyfile(tf, f)
                self.states.tracked_files.add(f)
                print('new file. take from target')
            # 6. f only in lca.
            # file is removed. Do nothing
            elif f in lca_files:
                print('file is removed')
            else:
                print('I worte a bug!')

        # create a merge commit which has 2 parents
        self.__merge_commit(self.states.head, node_id)

    def log(self):
        def tree_recursion(node_id):
            if node_id is None:
                return

            node = self.load_node(node_id)

            print('======================')
            print('node id/message: ', node.id)
            print('blobs:', node.blobs)
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

        def node_info_str(node):
            return '{}'.format(node.id)

        def get_edges(node_hash):
            node = self.load_node(node_hash)

            edges = [(node_info_str(node), node_info_str(self.load_node(c)))
                     for c in node.children]
            for c in node.children:
                edges += get_edges(c)
            return edges

        graph = nx.DiGraph()

        edges = get_edges(self.root_name)
        edges.append(('HEAD', node_info_str(self.load_node(self.states.head))))

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

    argsp = argsubparsers.add_parser('add')
    argsp.add_argument('files', nargs='+')

    argsp = argsubparsers.add_parser('merge')
    argsp.add_argument('id',
                       help='target commit')

    argsp = argsubparsers.add_parser('commit')
    argsp.add_argument('message',
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
        git.commit(args.message)
    elif args.command == 'init':
        git.init()
    elif args.command == 'log':
        git.log()
    elif args.command == 'gitk':
        git.gitk()
    elif args.command == 'merge':
        git.merge(args.id)
    elif args.command == 'add':
        git.add(args.files)
