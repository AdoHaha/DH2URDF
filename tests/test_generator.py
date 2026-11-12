import json
import math
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
NODE = """
const fs = require('node:fs');
const { generateURDF } = require('./dh-converter.js');
const [table, name] = JSON.parse(fs.readFileSync(0, 'utf8'));
try { process.stdout.write(generateURDF(table, name)); }
catch (error) { process.stderr.write(error.message); process.exitCode = 1; }
"""


def generate(table, name='test_robot'):
    return subprocess.run(
        ['node', '-e', NODE], input=json.dumps([table, name]), text=True,
        cwd=ROOT, capture_output=True,
    )


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def translation(x=0, y=0, z=0):
    matrix = identity()
    matrix[0][3], matrix[1][3], matrix[2][3] = x, y, z
    return matrix


def rx(t):
    matrix = identity()
    matrix[1][1] = matrix[2][2] = math.cos(t)
    matrix[2][1], matrix[1][2] = math.sin(t), -math.sin(t)
    return matrix


def ry(t):
    matrix = identity()
    matrix[0][0] = matrix[2][2] = math.cos(t)
    matrix[0][2], matrix[2][0] = math.sin(t), -math.sin(t)
    return matrix


def rz(t):
    matrix = identity()
    matrix[0][0] = matrix[1][1] = math.cos(t)
    matrix[1][0], matrix[0][1] = math.sin(t), -math.sin(t)
    return matrix


def product(*matrices):
    result = identity()
    for matrix in matrices:
        result = multiply(result, matrix)
    return result


def generated_pose(joints, terminal, values):
    by_child = {joint.find('child').get('link'): joint for joint in joints}
    chain = []
    link = terminal
    seen = set()
    while link != 'base_link':
        if link in seen:
            raise AssertionError('Cycle in generated joint chain')
        seen.add(link)
        joint = by_child[link]
        chain.append(joint)
        link = joint.find('parent').get('link')
    chain.reverse()
    matrices = []
    for joint in chain:
        origin = joint.find('origin')
        x, y, z = map(float, origin.get('xyz').split())
        roll, pitch, yaw = map(float, origin.get('rpy').split())
        matrices.append(product(translation(x, y, z), rz(yaw), ry(pitch), rx(roll)))
        if joint.get('type') == 'revolute':
            matrices.append(rz(values[joint.get('name')]))
        elif joint.get('type') == 'prismatic':
            matrices.append(translation(z=values[joint.get('name')]))
    return product(*matrices)


class GeneratorTests(unittest.TestCase):
    def test_classic_headers_and_connected_transforms(self):
        rows = [
            {'th': .4, 'd': .7, 'a': .5, 'alpha': -.6, 'type': 'R', 'q': .3},
            {'th': -.2, 'd': .3, 'a': -.8, 'alpha': .5, 'type': 'P', 'q': -.4},
            {'th': .9, 'd': -.2, 'a': .2, 'alpha': .1, 'type': 'R', 'q': -.5},
        ]
        headers = {
            'standard': '|θᵢ|dᵢ|aᵢ|αᵢ|joint|',
            'modified': '|aᵢ₋₁|αᵢ₋₁|θᵢ|dᵢ|joint|',
        }
        for convention, header in headers.items():
            with self.subTest(convention=convention):
                text = [header, '|---|---|---|---|---|']
                for row in rows:
                    cells = [row[key] for key in (
                        ('th', 'd', 'a', 'alpha', 'type') if convention == 'standard'
                        else ('a', 'alpha', 'th', 'd', 'type')
                    )]
                    text.append('|' + '|'.join(map(str, cells)) + '|')
                generated = generate('\n'.join(text))
                self.assertEqual(generated.returncode, 0, generated.stderr)
                robot = ET.fromstring(generated.stdout)
                links, joints = robot.findall('link'), robot.findall('joint')
                self.assertEqual(len(links), 1 + 2 * len(rows))
                self.assertEqual(len(joints), 2 * len(rows))
                names = {link.get('name') for link in links}
                self.assertEqual(len(names), len(links))
                expected_axes = {
                    'x_red': ((1, 0, 0), (1, 0, 0, 1)),
                    'y_green': ((0, 1, 0), (0, 1, 0, 1)),
                    'z_blue': ((0, 0, 1), (0, 0, 1, 1)),
                }
                for link in links:
                    if link.get('name') not in {'base_link', *(f'link{i}' for i in range(1, len(rows) + 1))}:
                        continue
                    visuals = link.findall('visual')
                    self.assertEqual(len(visuals), 3)
                    for visual in visuals:
                        material = visual.find('material')
                        axis, color = expected_axes[material.get('name')]
                        self.assertEqual(tuple(map(float, material.find('color').get('rgba').split())), color)
                        origin = visual.find('origin')
                        center = tuple(map(float, origin.get('xyz').split()))
                        for actual, expected in zip(center, (0.125 * value for value in axis)):
                            self.assertAlmostEqual(actual, expected)
                        roll, pitch, yaw = map(float, origin.get('rpy').split())
                        rotation = product(rz(yaw), ry(pitch), rx(roll))
                        for actual, expected in zip((rotation[i][2] for i in range(3)), axis):
                            self.assertAlmostEqual(actual, expected)
                        cylinder = visual.find('geometry/cylinder')
                        self.assertEqual(cylinder.get('length'), '0.25')
                        self.assertEqual(cylinder.get('radius'), '0.012')
                parents = {}
                for joint in joints:
                    parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
                    self.assertIn(parent, names)
                    self.assertIn(child, names)
                    self.assertNotIn(child, parents)
                    parents[child] = parent
                self.assertEqual(set(parents), names - {'base_link'})
                for link in names - {'base_link'}:
                    seen = set()
                    while link != 'base_link':
                        self.assertNotIn(link, seen, 'Generated joint cycle')
                        seen.add(link)
                        link = parents[link]
                values = {f'joint{i}': row['q'] for i, row in enumerate(rows, 1)}
                expected = identity()
                for i, row in enumerate(rows, 1):
                    angle = row['th'] + (row['q'] if row['type'] == 'R' else 0)
                    distance = row['d'] + (row['q'] if row['type'] == 'P' else 0)
                    z = product(rz(angle), translation(z=distance))
                    x = product(translation(x=row['a']), rx(row['alpha']))
                    expected = multiply(expected, product(z, x) if convention == 'standard' else product(x, z))
                    actual = generated_pose(joints, f'link{i}', values)
                    for actual_row, expected_row in zip(actual, expected):
                        for actual_value, expected_value in zip(actual_row, expected_row):
                            self.assertAlmostEqual(actual_value, expected_value, places=12)

    def test_input_validation_and_short_headers(self):
        self.assertEqual(generate('|th|d|a|alpha|R|\n|pi/2|1e-3|2|0|true|').returncode, 0)
        self.assertEqual(generate('|a|alpha|th|d|R|\n|2|0|pi/2|1|false|').returncode, 0)
        self.assertEqual(generate('|theta_i|d_i|a_i|alpha_i|joint|\n|0|0|0|0|R|').returncode, 0)
        self.assertEqual(generate('|a_{i-1}|alpha_{i-1}|theta_i|d_i|joint|\n|0|0|0|0|P|').returncode, 0)
        bad = [
            ('|th|d|a|alpha|R|\n|0|0|1|0|maybe|', 'Line 2'),
            ('|th|d|a|alpha|R|\n|0|0|1/0|0|R|', 'Line 2'),
            ('|th|d|a|alpha|R|\n|0|0|process.exit(1)|0|R|', 'Line 2'),
            ('|th|d|a|alpha|R|\n|0|0|0|R|', 'Line 2'),
            ('|th|d|a|alpha|R|', 'at least one'),
        ]
        for table, message in bad:
            with self.subTest(table=table):
                result = generate(table)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(message, result.stderr)
        self.assertIn('Robot name', generate('|th|d|a|alpha|R|\n|0|0|0|0|R|', '<bad>').stderr)


if __name__ == '__main__':
    unittest.main()
