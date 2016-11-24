Converting from D-H notation to URDF files
----------------------


This file converts files in format of Markdown table into a URDF (universal robot description format) file representing of kinematic chain.
Tables should have a format of, with R being whether a joint is Rotational (true| false). Header is not nessessary.

| theta | d | a | alpha |R  |
|-------|---|---|-------|---|
|       |   |   |       |   |
|       |   |   |       |   |
|       |   |   |       |   |

Generally as this notation translates into series of transformations Rot(z,theta)Trans(z,d)Trans(x,a)Rot(x,alpha) two joints -- active and fixed are needed
for each row of the table. 
	
	window.DH_converter = window.DH_converter||{};	

	class window.Robot_Maker
		
		constructor: (@table) ->
			@parser= new DOMParser();
			@robot_dom= @parser.parseFromString(@robot_xml,"text/xml");
			@urdf=@convert_text_table(@table)
		robot_xml : "<robot name='example_robot'>
			<link name='link0_passive'><visual>
			<material name='blue'><color rgba='0 0 .8 1'/></material>
			<geometry> <origin xyz='0 0 0' rpy='0 0 0'/>
			
			<cylinder length='0.6' radius='0.1'/></geometry></visual></link>
			
			<link name='link0_x_axis'><visual>
			 <material name='red'><color rgba='1 0 0 1'/></material>
			<geometry> <origin xyz='0 0 0' rpy='0 0 0'/>
			<cylinder length='0.6' radius='0.1'/></geometry></visual></link>
			<joint name='q0_x' type='fixed'>
			<origin xyz='0 0 0' rpy='0 1.571 0'/>
			<parent link='link0_passive' />
			<child link='link0_x_axis' />
			</joint>
			
			</robot> ";

A Moustache template is prepared to put the adequate values from the row od DH table

		row_template_insert:"
		<link name='link{{name}}'></link>
		<link name='link{{name}}_passive'><visual> <origin xyz='0 0 0.25' rpy='0 0 0'/><material name='blue' /><geometry><cylinder length='0.5' radius='0.05'/></geometry></visual></link>
		<link name='link{{name}}_x_axis'><visual><origin xyz='0 0 0.25' rpy='0 0 0'/> <material name='red' /><geometry><cylinder length='0.5' radius='0.05'/></geometry></visual></link>
		
	
		<joint name='q{{name}}' type='{{type}}'>
		<origin xyz='0 0 {{d}}' rpy='0 0 {{th}}'/>
		<parent link='link{{previous_name}}_passive' />
		<child link='link{{name}}' />
		<axis xyz='0 0 1'/>
		</joint>
		
		
		<joint name='q{{row_no}}_passive' type='fixed'>
		<origin xyz='{{a}} 0 0' rpy='{{alpha}} 0 0'/>
		<parent link='link{{name}}' />
		<child link='link{{name}}_passive' />
		</joint>
		
		<joint name='q{{row_no}}_x' type='fixed'>
		<origin xyz='0 0 0' rpy='0 1.571 0'/>
		<parent link='link{{name}}_passive' />
		<child link='link{{name}}_x_axis' />
		</joint>
		";	


Using regex, we parse rows of table, starting from 2nd row. Regex looks for floating values for every but last column, we convert last column to boolean value.
Then, each row is passed to DH_row_to_links function

		convert_text_table: (table) ->
	
			### function converts markdown table into nodes, we assume that table has header (two lines)
			###
		
			pattern=/\|\W*?([-+]?[0-9]*\.?[0-9]+)\W*?\|\W*?([-+]?[0-9]*\.?[0-9]+)\W*?\|\W*?([-+]?[0-9]*\.?[0-9]+)\W*?\|\W*?([-+]?[0-9]*\.?[0-9]+)\W*?\|\W*?(\w+?)\W*?\|/;
			regg=new RegExp(pattern)
			lines_of_text=table.split('\n');
			robot_dict={}
			sam_robot = @robot_dom.getElementsByTagName("robot")[0]
			for line_no in [2...lines_of_text.length]
	 			robot_dict.row_no=line_no-1
	 			przerob_linijke=lines_of_text[line_no].match(regg)
	 			robot_dict.th=przerob_linijke[1]
	 			robot_dict.d=przerob_linijke[2]
	 			robot_dict.a=przerob_linijke[3]
	 			robot_dict.alpha=przerob_linijke[4]
	 			robot_dict.R=przerob_linijke[5]=="true"
	 			wynik=@DH_row_to_links(robot_dict)
	 			
	 			sam_robot.insertAdjacentHTML('beforeend',wynik)
				console.log(sam_robot)
			return sam_robot.outerHTML
			
Function below takes a dictionary of row values and converts it into xml code

	
		DH_row_to_links: (row_dict) ->
			### function gets a dictionary representing row in DH table, returns xml node
			###
			row_no=row_dict.row_no
			row_dict.name=row_no
			row_dict.previous_name=row_no-1
			if row_dict.R
				row_dict.type="revolute"
			else
				row_dict.type="prismatic"
			
			row_xml=Mustache.render(@row_template_insert,row_dict)
			return 	row_xml
		
	
		
		
		
		
		
	
