(function (root) {
  'use strict';

  function expression(source, line) {
    const input = source.replace(/\s+/g, '');
    const tokens = input.match(/(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?|pi|[()+*/-]/gi) || [];
    if (!input || tokens.join('').toLowerCase() !== input.toLowerCase()) {
      throw new Error(`Line ${line}: invalid number or expression "${source}".`);
    }
    let at = 0;
    function atom() {
      const token = tokens[at++];
      if (token === '+' || token === '-') return (token === '-' ? -1 : 1) * atom();
      if (token === '(') {
        const value = sum();
        if (tokens[at++] !== ')') throw new Error(`Line ${line}: missing closing parenthesis.`);
        return value;
      }
      if (token && token.toLowerCase() === 'pi') return Math.PI;
      if (token && /^(?:\d|\.)/.test(token)) return Number(token);
      throw new Error(`Line ${line}: invalid expression "${source}".`);
    }
    function product() {
      let value = atom();
      while (tokens[at] === '*' || tokens[at] === '/') {
        const operator = tokens[at++];
        const next = atom();
        value = operator === '*' ? value * next : value / next;
      }
      return value;
    }
    function sum() {
      let value = product();
      while (tokens[at] === '+' || tokens[at] === '-') {
        const operator = tokens[at++];
        const next = product();
        value = operator === '+' ? value + next : value - next;
      }
      return value;
    }
    const value = sum();
    if (at !== tokens.length || !Number.isFinite(value)) {
      throw new Error(`Line ${line}: invalid or non-finite expression "${source}".`);
    }
    return Object.is(value, -0) ? 0 : value;
  }

  function columns(line) {
    return line.trim().replace(/^\|/, '').replace(/\|$/, '').split('|').map(cell => cell.trim());
  }

  function headerName(cell) {
    return cell.toLowerCase().replace(/θ/g, 'theta').replace(/α/g, 'alpha')
      .replace(/ᵢ₋₁/g, 'prev').replace(/ᵢ/g, '_i')
      .replace(/\{i-1\}|\(i-1\)/g, 'prev')
      .replace(/_/g, '');
  }

  function parseTable(table) {
    const lines = table.split(/\r?\n/).map((text, index) => ({ text: text.trim(), line: index + 1 }))
      .filter(entry => entry.text);
    if (!lines.length) throw new Error('Enter a DH table with a header and at least one row.');
    const header = columns(lines[0].text).map(headerName);
    const is = (index, names) => names.includes(header[index]);
    const jointHeader = is(4, ['r', 'joint', 'type']);
    const standard = header.length === 5 && jointHeader
      && is(0, ['th', 'theta', 'thetai']) && is(1, ['d', 'di'])
      && is(2, ['a', 'ai']) && is(3, ['alpha', 'alphai']);
    const modified = header.length === 5 && jointHeader
      && is(0, ['a', 'aprev']) && is(1, ['alpha', 'alphaprev'])
      && is(2, ['th', 'theta', 'thetai']) && is(3, ['d', 'di']);
    const convention = standard ? 'standard' : modified ? 'modified' : null;
    if (!convention) throw new Error('Header must use θᵢ, dᵢ, aᵢ, αᵢ, joint (standard) or aᵢ₋₁, αᵢ₋₁, θᵢ, dᵢ, joint (modified).');
    const rows = [];
    for (const entry of lines.slice(1)) {
      const cells = columns(entry.text);
      if (cells.every(cell => /^:?-{3,}:?$/.test(cell))) continue;
      if (cells.length !== 5) throw new Error(`Line ${entry.line}: expected five columns.`);
      const values = cells.slice(0, 4).map(cell => expression(cell, entry.line));
      const kind = cells[4].toLowerCase();
      if (!['true', 'false', 'r', 'p', 'revolute', 'prismatic'].includes(kind)) {
        throw new Error(`Line ${entry.line}: joint must be true/false, R/P, or revolute/prismatic.`);
      }
      const row = convention === 'standard'
        ? { theta: values[0], d: values[1], a: values[2], alpha: values[3] }
        : { a: values[0], alpha: values[1], theta: values[2], d: values[3] };
      row.type = ['true', 'r', 'revolute'].includes(kind) ? 'revolute' : 'prismatic';
      rows.push(row);
    }
    if (!rows.length) throw new Error('Add at least one DH row below the header.');
    return { convention, rows };
  }

  function frame(linkName) {
    return [
      `  <link name="${linkName}">`,
      '    <visual><origin xyz="0 0 0.125" rpy="0 0 0"/><geometry><cylinder length="0.25" radius="0.012"/></geometry><material name="z_blue"><color rgba="0.1 0.25 0.9 1"/></material></visual>',
      '    <visual><origin xyz="0.125 0 0" rpy="0 1.5707963267948966 0"/><geometry><cylinder length="0.25" radius="0.012"/></geometry><material name="x_red"><color rgba="0.9 0.15 0.15 1"/></material></visual>',
      '  </link>'
    ];
  }

  function joint(name, type, parent, child, xyz, rpy) {
    const result = [
      `  <joint name="${name}" type="${type}">`,
      `    <parent link="${parent}"/><child link="${child}"/>`,
      `    <origin xyz="${xyz}" rpy="${rpy}"/>`
    ];
    if (type !== 'fixed') {
      result.push('    <axis xyz="0 0 1"/>');
      // Editable starting ranges for the ModelRobot sliders, not physical limits.
      result.push(type === 'revolute'
        ? '    <limit lower="-3.141592653589793" upper="3.141592653589793" effort="1" velocity="1"/>'
        : '    <limit lower="-1" upper="1" effort="1" velocity="1"/>');
    }
    result.push('  </joint>');
    return result;
  }

  function generateURDF(table, name = 'dh_robot') {
    if (!/^[A-Za-z][A-Za-z0-9_]*$/.test(name)) {
      throw new Error('Robot name must start with a letter and contain only letters, digits, or underscores.');
    }
    const { convention, rows } = parseTable(table);
    const result = ['<?xml version="1.0" encoding="UTF-8"?>', `<robot name="${name}">`, ...frame('base_link')];
    rows.forEach((row, index) => {
      const number = index + 1;
      const parent = index ? `link${index}` : 'base_link';
      const terminal = `link${number}`;
      const intermediate = `link${number}_frame`;
      const fixedXYZ = `${row.a} 0 0`;
      const fixedRPY = `${row.alpha} 0 0`;
      const movingXYZ = `0 0 ${row.d}`;
      const movingRPY = `0 0 ${row.theta}`;
      result.push(...frame(terminal), `  <link name="${intermediate}"/>`);
      if (convention === 'standard') {
        result.push(...joint(`joint${number}`, row.type, parent, intermediate, movingXYZ, movingRPY));
        result.push(...joint(`joint${number}_frame`, 'fixed', intermediate, terminal, fixedXYZ, fixedRPY));
      } else {
        result.push(...joint(`joint${number}_frame`, 'fixed', parent, intermediate, fixedXYZ, fixedRPY));
        result.push(...joint(`joint${number}`, row.type, intermediate, terminal, movingXYZ, movingRPY));
      }
    });
    result.push('</robot>');
    return result.join('\n') + '\n';
  }

  class Robot_Maker {
    constructor(table, name) { this.urdf = generateURDF(table, name); }
  }

  root.DH_converter = { parseTable, generateURDF };
  root.Robot_Maker = Robot_Maker;
  if (typeof module !== 'undefined' && module.exports) module.exports = root.DH_converter;
})(typeof globalThis !== 'undefined' ? globalThis : window);
