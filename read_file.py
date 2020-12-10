import numpy as np

with open('t1_straight_line_no_data.txt', 'r') as f:
	line = f.readline();

	dt_array = []
	x_prior_array = []
	p_prior_array = []
	x_post_array = []
	p_post_array = []

	for idx, line in enumerate(f):
		dt,x_prior,p_prior,x_post, p_post = line.replace('\n', '').split(';');
		x_prior = x_prior[:-1].split(',')
		p_prior = p_prior[:-1].split(',')
		x_post = x_post[:-1].split(',')
		p_post = p_post[:-1].split(',')

		dt_array.append(dt)
		x_prior_array.append(x_prior)
		p_prior_array.append(p_prior)
		x_post_array.append(x_post)
		p_post_array.append(p_post)
		# if (idx>3):
		# 	break;

	dt_array = np.array(dt_array)
	x_prior_array = np.array(x_prior_array)
	p_prior_array = np.array(p_prior_array)
	x_post_array = np.array(x_post_array)
	p_post_array = np.array(p_post_array)

	print ("dt", dt_array)
	print ("x_prior", x_prior_array)
	print ("p_prior", p_prior_array)
	print ("x_post", x_post_array)
	print ("p_post", len(p_post_array))
		


