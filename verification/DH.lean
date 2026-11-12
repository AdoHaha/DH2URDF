/-
Composition is associative, and rotations about z commute with translations
along z. No commutation involving an x transform is assumed.
-/

variable {G : Type}
variable (compose : G → G → G)
variable (assoc : ∀ a b c : G,
  compose (compose a b) c = compose a (compose b c))
include assoc

private theorem swap_z (d r q : G)
    (dr : compose d r = compose r d)
    (dq : compose d q = compose q d) :
    compose (compose d r) q = compose (compose r q) d := by
  calc
    compose (compose d r) q = compose (compose r d) q := congrArg (fun v ↦ compose v q) dr
    _ = compose r (compose d q) := assoc r d q
    _ = compose r (compose q d) := congrArg (compose r) dq
    _ = compose (compose r q) d := (assoc r q d).symm

/-- Standard revolute DH: URDF origin Tz(d) Rz(θ), then Rz(q), then Tx(a) Rx(α). -/
theorem standard_revolute_order (d r q x : G)
    (dr : compose d r = compose r d)
    (dq : compose d q = compose q d) :
    compose (compose (compose d r) q) x =
      compose (compose (compose r q) d) x :=
  congrArg (fun v ↦ compose v x) (swap_z compose assoc d r q dr dq)

omit assoc in
/-- Standard prismatic DH: the variable z translation follows the origin. -/
theorem standard_prismatic_order (d r q x : G)
    (dr : compose d r = compose r d) :
    compose (compose (compose d r) q) x =
      compose (compose (compose r d) q) x :=
  congrArg (fun v ↦ compose (compose v q) x) dr

/-- Modified revolute DH puts the fixed X transform before the moving joint. -/
theorem modified_revolute_order (x d r q : G)
    (dr : compose d r = compose r d)
    (dq : compose d q = compose q d) :
    compose x (compose (compose d r) q) =
      compose x (compose (compose r q) d) :=
  congrArg (compose x) (swap_z compose assoc d r q dr dq)

omit assoc in
/-- Modified prismatic DH puts the fixed X transform before the moving joint. -/
theorem modified_prismatic_order (x d r q : G)
    (dr : compose d r = compose r d) :
    compose x (compose (compose d r) q) =
      compose x (compose (compose r d) q) :=
  congrArg (fun v ↦ compose x (compose v q)) dr

omit assoc in
/-- Equal row transforms give equal whole-chain transforms for any number of rows. -/
theorem chain_order (rows : List (G × G)) (base : G)
    (row_equal : ∀ row ∈ rows, row.1 = row.2) :
    rows.foldl (fun pose row ↦ compose pose row.1) base =
      rows.foldl (fun pose row ↦ compose pose row.2) base := by
  induction rows generalizing base with
  | nil => rfl
  | cons row rest ih =>
    simp only [List.foldl_cons]
    rw [row_equal row (by simp)]
    exact ih (compose base row.2) (fun other member ↦ row_equal other (by simp [member]))
