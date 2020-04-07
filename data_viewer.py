from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *

class data_viewer:
    def __init__(self):
        time_window_length=100
        self.plotter = Plotter(plotting_frequency=100, # refresh plot every 100 time steps
                               time_window=time_window_length)  # plot last time_window seconds of data
        # set up the plot window
        # define first row
        pn_plots = PlotboxArgs(plots=['pn'],
                               labels={'left': 'pn(m)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        pe_plots = PlotboxArgs(plots=['pe'],
                               labels={'left': 'pe(m)', 'bottom': 'Time (s)'},
                               time_window=time_window_length)
        h_plots = PlotboxArgs(plots=['h'],
                              labels={'left': 'h(m)', 'bottom': 'Time (s)'},
                              time_window=time_window_length)
        first_row = [pn_plots, pe_plots, h_plots]

        u_plots = PlotboxArgs(plots='u', labels={'left': 'u(m/s)', 'bottom': 'Time (s)'}, time_window=time_window_length)
        v_plots = PlotboxArgs(plots='v', labels={'left': 'v(m/s)', 'bottom': 'Time (s)'}, time_window=time_window_length)
        w_plots = PlotboxArgs(plots='w', labels={'left': 'w(m/s)', 'bottom': 'Time (s)'}, time_window=time_window_length)
        second_row = [u_plots, v_plots, w_plots]

        # define second row
        phi_plots = PlotboxArgs(plots=['phi'],
                                labels={'left': 'phi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True,
                                time_window=time_window_length)
        theta_plots = PlotboxArgs(plots=['theta'],
                                  labels={'left': 'theta(deg)', 'bottom': 'Time (s)'},
                                  rad2deg=True,
                                  time_window=time_window_length)
        psi_plots = PlotboxArgs(plots=['psi'],
                                labels={'left': 'psi(deg)', 'bottom': 'Time (s)'},
                                rad2deg=True,
                                time_window=time_window_length)
        third_row = [phi_plots, theta_plots, psi_plots]

        # define third row
        p_plots = PlotboxArgs(plots=['p'],
                              labels={'left': 'p(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        q_plots = PlotboxArgs(plots=['q'],
                              labels={'left': 'q(deg/s)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        r_plots = PlotboxArgs(plots=['r'],
                              labels={'left': 'r(deg)', 'bottom': 'Time (s)'},
                              rad2deg=True,
                              time_window=time_window_length)
        fourth_row = [p_plots, q_plots, r_plots]
        plots = [first_row,
                 second_row,
                 third_row,
                 fourth_row
                 ]
        # Add plots to the window
        self.plotter.add_plotboxes(plots)
        # Define and label vectors for more convenient/natural data input
        self.plotter.define_input_vector('true_state', ['pn', 'pe', 'h', 'u', 'v', 'w', 'phi', 'theta', 'psi',
                                                        'p', 'q', 'r'])
        # plot timer
        self.time = 0.

    def update(self, true_state, ts):
        ## Add the state data in vectors
        # the order has to match the order in lines 72-76
        true_state_list = true_state.tolist()
        self.plotter.add_vector_measurement('true_state', true_state_list, self.time)

        # Update and display the plot
        self.plotter.update_plots()

        # increment time
        self.time += ts