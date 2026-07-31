// troika-three-utils ships no type declarations and there's no @types package
// for it. Minimal shape for the one function we actually use - see
// node_modules/troika-three-utils/src/DerivedMaterial.js for the full option
// list if more are needed later.
declare module 'troika-three-utils' {
	import type { Material } from 'three';

	export interface DerivedMaterialOptions {
		chained?: boolean;
		defines?: Record<string, unknown>;
		extensions?: Record<string, boolean>;
		uniforms?: Record<string, { value: unknown }>;
		timeUniform?: string;
		vertexDefs?: string;
		vertexMainIntro?: string;
		vertexMainOutro?: string;
		vertexTransform?: string;
		fragmentDefs?: string;
		fragmentMainIntro?: string;
		fragmentMainOutro?: string;
		fragmentColorTransform?: string;
		customRewriter?: (shaders: { vertexShader: string; fragmentShader: string }) => {
			vertexShader: string;
			fragmentShader: string;
		};
	}

	export function createDerivedMaterial<T extends Material>(
		baseMaterial: T,
		options: DerivedMaterialOptions
	): T;
}
