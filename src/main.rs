

/*pub fn input_handler(
    button: PinDriver<'static, Gpio14, Input>, 
) -> impl FnMut(Duration, &mut CalculatorInput) {}*/

mod calculator {
    use core::f64;

    #[derive(Default)]
    pub struct CalculatorState {
        sigma: f64,
        clock: f64,
        orientation: (f64, f64, f64), 
    }

    impl CalculatorState {
        pub fn new() -> Self {
            Self {
                sigma: f64::INFINITY,
                clock: 0.0,
                orientation: (0.0,0.0,0.0), //at the beginning it's the orientation of reference or 0,0,0
            }
        }
    }

    //Quaternion {s: w, // scalare
    //            v: Vector3 {x: x, y: y, z: z, }}

    xdevs::component!(
        ident = Calculator,
        input= {
            data<mint::Quaternion<f64>>,//??????
        },
        output = {
            position<(f64, f64, f64)>,
        },
        state = CalculatorState,
    );

    impl xdevs::Atomic for Calculator {
        fn delta_int(state: &mut Self::State) {
            //wait to have extern values from sensor
            state.clock += state.sigma;
            println!("Waiting data t={} ", state.clock);
            state.sigma = f64::INFINITY;
        }

        fn lambda(state: &Self::State, output: &mut Self::Output) {
            //put the value of orientation on output
            output.position.add_value(state.orientation);
            println!("Orientation sent !")
        }

        fn ta(state: &Self::State) -> f64 {
            state.sigma
        }

        fn delta_ext(state: &mut Self::State, e: f64, input: &Self::Input) {
            state.sigma -= e;
            state.clock += e;
            //quaternion register : 8bytes -> wLSB, wMSB, xLSB, xMSB, yLSB, yMSB, zLSB, zMSB
            /*quaternion form : Quaternion {s: w, // scalare
                                            v: Vector3 {x: x, y: y, z: z, }}*/
            
            let values = input.data.get_values();
            if !values.is_empty() {
                //quat is not empty so we take the values 
                let quat = values[0]; // first element
                let w = quat.s;
                let x = quat.v.x;
                let y = quat.v.y;
                let z = quat.v.z;
                println!("Quaternion : w={}, x={}, y={}, z={}", w, x, y, z);

                //calculate euler angle thanks to quaternion -> calculate roll, pitch, yaw
                //roll : rotation around the longitudinal axis (nose to tail)
                //pitch : rotation around the lateral axis (wing to wing)
                //yaw : rotation around the vertical axis (top to bottom)
                let roll = (2.0_f64*(w*x + y*z)).atan2(1.0_f64 - 2.0_f64*(x*x + y*y)); //_f64 to precise what type we are using
                let pitch = (2.0_f64*(w*y - z*x)).asin();
                let yaw = (2.0_f64*(w*z + x*y)).atan2(1.0 - 2.0*(y*y + z*z));
                state.orientation = (pitch, roll, yaw);
                //println!("Orientation : {}", state.orientation);
            }
            state.sigma = 0.;
        }
    }
}

mod analyser {
    use core::f64;

    #[derive(Default)]
    pub struct AnalyserState {
        sigma: f64,
        clock: f64,
        orientation_ref: (f64, f64, f64), //to initialize in main
        tolerance: f64, //to initialize in main
    }

    impl AnalyserState {
        pub fn new(orientation_ref:(f64, f64, f64), tolerance:f64) -> Self {
            Self {
                sigma: f64::INFINITY,
                clock: 0.0,
                orientation_ref,
                tolerance,
            }
        }
    }

    xdevs::component!(
        ident = Analyser,
        input= {
            position<(f64, f64, f64)>,
        },
        output = {
            action<char>,
        },
        state = AnalyserState,
    );

    impl xdevs::Atomic for Analyser {
        fn delta_int(state: &mut Self::State) {
            //wait to have extern values from the calculator
            state.clock += state.sigma;
            println!("waiting orientation t={} ", state.clock);
            state.sigma = f64::INFINITY;
        }

        fn lambda(state: &Self::State, output: &mut Self::Output) {
            //put the instruction for the motor in output
            
        }

        fn ta(state: &Self::State) -> f64 {
            state.sigma
        }

        fn delta_ext(state: &mut Self::State, e: f64, input: &Self::Input) {
            state.sigma -= e;
            state.clock += e;
            //recup orientation value from input
            let values = input.position.get_values();
            if !values.is_empty() {
                //input port is not empty so we take the values 
                let (roll, pitch, yaw) = values[0];  // first element of list

                let (roll_ref, pitch_ref, yaw_ref) = state.orientation_ref;
                //calculate the absolut difference
                let roll_diff = (roll - roll_ref).abs();
                let pitch_diff = (pitch - pitch_ref).abs();
                let yaw_diff = (yaw - yaw_ref).abs();

                if roll_diff>=state.tolerance || pitch_diff>=state.tolerance || yaw_diff>=state.tolerance {
                    println!("Problem of position : satellite needs to move -> activation of motor");
                    //how to know what to do with the motor??
                }
            }
        }
    }
}

xdevs::component!(
    ident = CalculatorAnalyser,
    input = {
        data<mint::Quaternion<f64>>,
    },
    output = {
        action<char>,
    },
    components = {
        calculator: calculator::Calculator,
        analyser:analyser::Analyser,
    },
    couplings = {
        data -> calculator.data,
        calculator.position -> analyser.position,
        analyser.action -> action,
    }
);

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");

    let orientation_ref = (0., 0., 0.); //complete
    let tolerance = 1.0; //complete

    let calculator = calculator::Calculator::new(calculator::CalculatorState::new());
    let analyser = analyser::Analyser::new(analyser::AnalyserState::new(orientation_ref, tolerance));

    let calculatoranalyser = CalculatorAnalyser::new(calculator, analyser);

    let mut simulator = xdevs::simulator::Simulator::new(calculatoranalyser);
    let config = xdevs::simulator::Config::new(0.0, 60.0, 1.0, None);
    simulator.simulate_vt(&config);
}
