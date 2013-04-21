namespace Pid

type Constants = {
    ProportionalConstant : float
    IntegralConstant : float
    DifferentialConstant : float
}

module Core =
    type InitialState = {
        controlOutput : float
        setPoint : float
    }

    type State =
        {
            Output : float
            SetPoint : float
            PVn_1 : float option
            PVn_2 : float option
            lastChangedAt : System.DateTime
        }
        
        static member Initial(seed : InitialState) =
            {   Output = seed.controlOutput;
                SetPoint = seed.setPoint;
                PVn_1 = None;
                PVn_2 = None;
                lastChangedAt = System.DateTime.Now   }

        /// <param name="PV">The process variable, or observation of the controlled system.<param>
        member this.appliedTo (PV : float) (constants : Constants) =
            let COn_1 = this.Output
            let CO =
                let T = float (System.DateTime.Now - this.lastChangedAt).Milliseconds
                in COn_1
                    -   // Proportional term
                        let Kp = constants.ProportionalConstant
                        in (match this.PVn_1 with None -> 0.0 | Some pvn_1 -> Kp * (PV - pvn_1))

                    +   // Integral term
                        let Ki = constants.IntegralConstant
                        let error = this.SetPoint - PV
                        in (Ki * T * error)
                
                    -   // Differential term
                        let Kd = constants.DifferentialConstant
                        in (match this.PVn_1, this.PVn_2 with
                            | Some pvn_1, Some pvn_2 -> Kd / T * (PV - 2.0*pvn_1 + pvn_2)
                            | _, _ -> 0.0
                        )
            in { this with Output = CO; PVn_1 = Some PV; PVn_2 = this.PVn_1; lastChangedAt = System.DateTime.Now }